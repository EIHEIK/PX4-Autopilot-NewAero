#!/usr/bin/env python3
"""Upload a QGroundControl .plan mission to a running PX4 over MAVLink."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pymavlink import mavutil


GLOBAL_FRAMES = {
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
}


def number(value: object) -> float:
    return float("nan") if value is None else float(value)


def load_items(path: Path) -> list[dict]:
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    if payload.get("fileType") != "Plan":
        raise ValueError(f"{path} is not a QGroundControl Plan file")

    result = []
    for sequence, item in enumerate(payload.get("mission", {}).get("items", [])):
        if item.get("type") != "SimpleItem":
            raise ValueError(f"mission item {sequence} is not a SimpleItem")
        params = item.get("params", [])
        if len(params) != 7:
            raise ValueError(f"mission item {sequence} must contain seven parameters")
        frame = int(item["frame"])
        x = int(round(number(params[4]) * 1e7)) if frame in GLOBAL_FRAMES else int(number(params[4]))
        y = int(round(number(params[5]) * 1e7)) if frame in GLOBAL_FRAMES else int(number(params[5]))
        result.append({
            "sequence": sequence,
            "frame": frame,
            "command": int(item["command"]),
            "autocontinue": int(bool(item.get("autoContinue", True))),
            "param1": number(params[0]),
            "param2": number(params[1]),
            "param3": number(params[2]),
            "param4": number(params[3]),
            "x": x,
            "y": y,
            "z": number(params[6]),
        })
    if not result:
        raise ValueError("plan contains no mission items")
    return result


def heartbeat(connection) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def reset_current_item(connection, target_system: int, target_component: int, timeout_s: float) -> None:
    """Select the first mission item after upload and verify PX4 accepted it."""
    connection.mav.mission_set_current_send(target_system, target_component, 0)
    deadline = time.monotonic() + min(timeout_s, 10.0)
    while time.monotonic() < deadline:
        message = connection.recv_match(type="MISSION_CURRENT", blocking=True, timeout=0.5)
        if message is not None and int(message.seq) == 0:
            return
    raise TimeoutError("mission uploaded, but PX4 did not confirm current mission item 0")


def upload(path: Path, port: int, timeout_s: float, clear_first: bool = False) -> None:
    items = load_items(path)
    connection = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{port}", source_system=250, source_component=190,
    )
    vehicle = connection.wait_heartbeat(timeout=timeout_s)
    if vehicle is None:
        raise TimeoutError(f"no PX4 heartbeat received on UDP {port}")
    target_system = vehicle.get_srcSystem()
    target_component = vehicle.get_srcComponent()

    heartbeat(connection)
    # MISSION_COUNT starts a replacement upload without invalidating the
    # currently active mission first. PX4 switches to the new dataman bank only
    # after a complete accepted transfer, so a timeout leaves the old mission
    # available. Keep explicit clearing as a recovery option, not the default.
    if clear_first:
        connection.mav.mission_clear_all_send(
            target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        clear_deadline = time.monotonic() + 5.0
        while time.monotonic() < clear_deadline:
            message = connection.recv_match(type="MISSION_ACK", blocking=True, timeout=0.5)
            if message is not None:
                if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"mission clear rejected with ACK {message.type}")
                break
        else:
            raise TimeoutError("mission clear was not acknowledged")

    connection.mav.mission_count_send(
        target_system, target_component, len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    sent: set[int] = set()
    deadline = time.monotonic() + timeout_s
    last_heartbeat = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_heartbeat >= 0.8:
            heartbeat(connection)
            last_heartbeat = now
        message = connection.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        if message.get_type() == "MISSION_ACK":
            if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"mission upload rejected with ACK {message.type}")
            if len(sent) == len(items):
                reset_current_item(connection, target_system, target_component, timeout_s)
                print(f"Uploaded {len(items)} mission items from {path}; current item reset to 0")
                return
            connection.mav.mission_count_send(
                target_system, target_component, len(items),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
            continue

        sequence = int(message.seq)
        if not 0 <= sequence < len(items):
            raise RuntimeError(f"PX4 requested invalid mission sequence {sequence}")
        item = items[sequence]
        connection.mav.mission_item_int_send(
            target_system,
            target_component,
            sequence,
            item["frame"],
            item["command"],
            1 if sequence == 0 else 0,
            item["autocontinue"],
            item["param1"],
            item["param2"],
            item["param3"],
            item["param4"],
            item["x"],
            item["y"],
            item["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        sent.add(sequence)
    raise TimeoutError(f"mission upload timed out after sending {sorted(sent)}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path, help="QGroundControl .plan file")
    parser.add_argument(
        "--port", type=int, default=14540,
        help="local receive port for PX4's Onboard MAVLink stream (default: 14540)",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--clear-first", action="store_true",
        help="explicitly erase the current vehicle mission before upload (normally unnecessary)",
    )
    arguments = parser.parse_args()
    upload(arguments.plan.resolve(), arguments.port, arguments.timeout, arguments.clear_first)


if __name__ == "__main__":
    main()
