#!/usr/bin/env python3
"""Compare QGC/PX4 heading, Gazebo truth heading and ground-track direction."""

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def dataset(ulog, name):
    return next(item for item in ulog.data_list if item.name == name and item.multi_id == 0)


def wrap_deg(values):
    return (values + 180.0) % 360.0 - 180.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--sequences", nargs="*", type=int, default=[10, 11, 12, 14, 16, 18])
    parser.add_argument("--min-groundspeed", type=float, default=20.0)
    parser.add_argument("--label", default="")
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    ulog = ULog(str(args.ulog))
    estimated = dataset(ulog, "vehicle_local_position")
    truth = dataset(ulog, "vehicle_local_position_groundtruth")
    mission = dataset(ulog, "mission_result")
    aero = dataset(ulog, "honghu_v8_aero_state")
    wind = dataset(ulog, "wind")

    te = estimated.data["timestamp"] * 1e-6
    tt = truth.data["timestamp"] * 1e-6
    tm = mission.data["timestamp"] * 1e-6
    ta = aero.data["timestamp"] * 1e-6
    tw = wind.data["timestamp"] * 1e-6
    estimated_heading = np.interp(tt, te, np.unwrap(estimated.data["heading"]))
    truth_heading = truth.data["heading"]
    course = np.arctan2(truth.data["vy"], truth.data["vx"])
    speed = np.hypot(truth.data["vx"], truth.data["vy"])
    sequence = mission.data["seq_current"][np.clip(
        np.searchsorted(tm, tt, side="right") - 1, 0, len(tm) - 1)]
    beta = np.interp(tt, ta, aero.data["beta_deg"])
    wind_north = np.interp(tt, tw, wind.data["windspeed_north"])
    wind_east = np.interp(tt, tw, wind.data["windspeed_east"])
    wind_speed = np.hypot(wind_north, wind_east)

    estimated_course = wrap_deg(np.degrees(estimated_heading - course))
    truth_course = wrap_deg(np.degrees(truth_heading - course))
    estimated_truth = wrap_deg(np.degrees(estimated_heading - truth_heading))
    rows = []
    for item in args.sequences:
        valid = (sequence == item) & (speed >= args.min_groundspeed)
        if not np.any(valid):
            continue
        rows.append({
            "mission_sequence": item,
            "time_start_s": float(tt[valid][0]),
            "time_end_s": float(tt[valid][-1]),
            "sample_count": int(np.count_nonzero(valid)),
            "estimated_heading_minus_course_mean_deg": float(np.mean(estimated_course[valid])),
            "truth_heading_minus_course_mean_deg": float(np.mean(truth_course[valid])),
            "estimated_heading_minus_truth_mean_deg": float(np.mean(estimated_truth[valid])),
            "estimated_heading_minus_truth_p95_abs_deg": float(
                np.percentile(np.abs(estimated_truth[valid]), 95)),
            "beta_mean_deg": float(np.mean(beta[valid])),
            "estimated_wind_mean_m_s": float(np.mean(wind_speed[valid])),
        })

    result = {
        "label": args.label,
        "ulog": str(args.ulog.resolve()),
        "minimum_groundspeed_m_s": args.min_groundspeed,
        "sequence_statistics": rows,
    }
    text = json.dumps(result, indent=2, ensure_ascii=False)
    print(text)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
