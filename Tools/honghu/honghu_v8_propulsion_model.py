#!/usr/bin/env python3
"""Executable reference model for the Honghu Wing V8 propulsion table.

The interpolation intentionally mirrors HonghuV8Common.cpp.  It is kept
separate from the ULog coefficient solver so propulsion subtraction and the
aerodynamic forward model remain independently testable.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TABLE = (
    ROOT
    / "simulation_models/models/honghu_wing_150kg_v8/propulsion_tables/propeller.csv"
)
KGF_TO_NEWTON = 9.80665


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def lerp(a: float, b: float, fraction: float) -> float:
    return a + (b - a) * fraction


def bracket(axis: Sequence[float], query: float) -> Tuple[int, int]:
    if len(axis) < 2:
        raise ValueError("interpolation axis must contain at least two values")
    high = 0
    while high < len(axis) and axis[high] <= query:
        high += 1
    high = max(1, min(high, len(axis) - 1))
    return high - 1, high


def fraction(low: float, high: float, value: float) -> float:
    return 0.0 if abs(high - low) < 1e-12 else (value - low) / (high - low)


@dataclass(frozen=True)
class PropulsionRow:
    altitude_m: float
    throttle_pct: float
    rpm: float
    airspeed_mps: float
    thrust_kgf: float
    torque_nm: float


@dataclass(frozen=True)
class PropulsionResult:
    rpm: float
    thrust_newton: float
    torque_nm: float
    clamped: bool


class HonghuV8PropulsionModel:
    """Three-dimensional altitude / throttle / airspeed table interpolation."""

    def __init__(self, table_path: Path = DEFAULT_TABLE):
        self.table_path = Path(table_path)
        with self.table_path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            self.rows = tuple(
                PropulsionRow(
                    float(row["altitude_m"]),
                    float(row["throttle_pct"]),
                    float(row["rpm"]),
                    float(row["airspeed_mps"]),
                    float(row["thrust_kgf"]),
                    float(row["torque_Nm"]),
                )
                for row in reader
            )
        self.altitudes = tuple(sorted({row.altitude_m for row in self.rows}))
        if len(self.altitudes) < 2 or not self.rows:
            raise ValueError(f"invalid propulsion table: {self.table_path}")

    def _rows_at(self, altitude_m: float, throttle_pct: float) -> list[PropulsionRow]:
        return sorted(
            (
                row
                for row in self.rows
                if row.altitude_m == altitude_m and row.throttle_pct == throttle_pct
            ),
            key=lambda row: row.airspeed_mps,
        )

    def _at_altitude(
        self, altitude_m: float, throttle_pct: float, airspeed_mps: float
    ) -> PropulsionResult:
        throttles = tuple(
            sorted(
                {0.0}
                | {
                    row.throttle_pct
                    for row in self.rows
                    if row.altitude_m == altitude_m
                }
            )
        )
        throttle = clamp(throttle_pct, 0.0, throttles[-1])
        t0, t1 = bracket(throttles, throttle)

        def at_level(level: float) -> PropulsionResult:
            if level == 0.0:
                return PropulsionResult(0.0, 0.0, 0.0, False)
            rows = self._rows_at(altitude_m, level)
            if len(rows) < 2:
                raise ValueError(
                    f"missing propulsion speed sweep at altitude={altitude_m}, throttle={level}"
                )
            speeds = tuple(row.airspeed_mps for row in rows)
            speed = clamp(airspeed_mps, speeds[0], speeds[-1])
            v0, v1 = bracket(speeds, speed)
            f = fraction(speeds[v0], speeds[v1], speed)
            return PropulsionResult(
                rows[v0].rpm,
                lerp(rows[v0].thrust_kgf, rows[v1].thrust_kgf, f) * KGF_TO_NEWTON,
                lerp(rows[v0].torque_nm, rows[v1].torque_nm, f),
                False,
            )

        low = at_level(throttles[t0])
        high = at_level(throttles[t1])
        f = fraction(throttles[t0], throttles[t1], throttle)
        return PropulsionResult(
            lerp(low.rpm, high.rpm, f),
            lerp(low.thrust_newton, high.thrust_newton, f),
            lerp(low.torque_nm, high.torque_nm, f),
            False,
        )

    def interpolate(
        self, altitude_m: float, throttle: float, airspeed_mps: float
    ) -> PropulsionResult:
        altitude = clamp(altitude_m, self.altitudes[0], self.altitudes[-1])
        speed = clamp(airspeed_mps, 0.0, 50.0)
        command = clamp(throttle, 0.0, 1.0)
        h0, h1 = bracket(self.altitudes, altitude)
        low = self._at_altitude(self.altitudes[h0], command * 100.0, speed)
        high = self._at_altitude(self.altitudes[h1], command * 100.0, speed)
        f = fraction(self.altitudes[h0], self.altitudes[h1], altitude)
        return PropulsionResult(
            lerp(low.rpm, high.rpm, f),
            lerp(low.thrust_newton, high.thrust_newton, f),
            lerp(low.torque_nm, high.torque_nm, f),
            altitude != altitude_m or speed != airspeed_mps or command != throttle,
        )

    def evaluate_many(
        self,
        altitude_m: Iterable[float],
        throttle: Iterable[float],
        airspeed_mps: Iterable[float],
    ) -> list[PropulsionResult]:
        return [
            self.interpolate(altitude, command, speed)
            for altitude, command, speed in zip(altitude_m, throttle, airspeed_mps)
        ]
