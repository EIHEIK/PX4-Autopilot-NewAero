#!/usr/bin/env python3
"""Executable reference model for the Honghu Wing V8 aerodynamics.

This module mirrors HonghuAeroV8.cpp deliberately.  It is dependency-free so
that table provenance, interpolation, control signs, trim and force/moment
conversion can be audited without starting Gazebo.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Dict, Iterable, Tuple


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TABLE_DIR = ROOT / "simulation_models/models/honghu_wing_150kg_v8/aero_tables"
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def lerp(a: float, b: float, fraction: float) -> float:
    return a + (b - a) * fraction


def smoothstep(low: float, high: float, value: float) -> float:
    x = clamp((value - low) / (high - low), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def isa_density(altitude_m: float) -> float:
    h = clamp(altitude_m, -500.0, 11000.0)
    ratio = max(0.1, 1.0 - 2.25577e-5 * h)
    return 1.225 * ratio**4.25588


class Grid2D:
    """CSV grid with the same edge clamping and bilinear interpolation as C++."""

    def __init__(self, path: Path):
        self.path = Path(path)
        lines = (
            line for line in self.path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        )
        reader = csv.reader(lines)
        header = next(reader)
        self.columns = tuple(float(value) for value in header[1:])
        rows = []
        values = []
        for row in reader:
            rows.append(float(row[0]))
            values.append(tuple(float(value) for value in row[1:]))
        self.rows = tuple(rows)
        self.values = tuple(values)
        self.validate()

    def validate(self) -> None:
        if len(self.rows) < 2 or len(self.columns) < 2:
            raise ValueError(f"{self.path}: grid must be at least 2x2")
        if any(b <= a for a, b in zip(self.rows, self.rows[1:])):
            raise ValueError(f"{self.path}: row axis is not strictly increasing")
        if any(b <= a for a, b in zip(self.columns, self.columns[1:])):
            raise ValueError(f"{self.path}: column axis is not strictly increasing")
        if len(self.values) != len(self.rows):
            raise ValueError(f"{self.path}: row/value count mismatch")
        if any(len(row) != len(self.columns) for row in self.values):
            raise ValueError(f"{self.path}: ragged grid")
        if any(not math.isfinite(value) for row in self.values for value in row):
            raise ValueError(f"{self.path}: non-finite coefficient")

    @staticmethod
    def _bracket(axis: Tuple[float, ...], query: float) -> Tuple[int, int]:
        for high in range(1, len(axis)):
            if query < axis[high]:
                return high - 1, high
        return len(axis) - 2, len(axis) - 1

    def interpolate(self, row_query: float, column_query: float) -> float:
        row = clamp(row_query, self.rows[0], self.rows[-1])
        column = clamp(column_query, self.columns[0], self.columns[-1])
        r0, r1 = self._bracket(self.rows, row)
        c0, c1 = self._bracket(self.columns, column)
        tr = (row - self.rows[r0]) / (self.rows[r1] - self.rows[r0])
        tc = (column - self.columns[c0]) / (self.columns[c1] - self.columns[c0])
        low = lerp(self.values[r0][c0], self.values[r0][c1], tc)
        high = lerp(self.values[r1][c0], self.values[r1][c1], tc)
        return lerp(low, high, tr)


@dataclass
class Coefficients:
    CL: float = 0.0
    CD: float = 0.0
    CY: float = 0.0
    Cl: float = 0.0
    Cm: float = 0.0
    Cn: float = 0.0

    def __iadd__(self, other: "Coefficients") -> "Coefficients":
        for item in fields(self):
            setattr(self, item.name, getattr(self, item.name) + getattr(other, item.name))
        return self

    def as_dict(self) -> Dict[str, float]:
        return {item.name: getattr(self, item.name) for item in fields(self)}


@dataclass(frozen=True)
class AeroResult:
    coefficients: Coefficients
    controls: Dict[str, Coefficients]
    force_frd_n: Tuple[float, float, float]
    moment_frd_nm: Tuple[float, float, float]
    force_gz_flu_n: Tuple[float, float, float]
    moment_gz_flu_nm: Tuple[float, float, float]
    rho_kg_m3: float
    qbar_pa: float


class HonghuV8AeroModel:
    AREA_M2 = 2.42
    SPAN_M = 3.96
    MAC_M = 0.62

    def __init__(self, table_dir: Path = DEFAULT_TABLE_DIR):
        self.table_dir = Path(table_dir)
        self.static = {
            name: Grid2D(self.table_dir / f"{name}.csv")
            for name in ("CL", "CD", "CY", "Cl", "Cm", "Cn")
        }
        control_dir = self.table_dir / "control_tables"
        self.control = {}
        for surface, names in {
            "canard": ("CL", "CD", "Cm"),
            "elevator": ("CL", "CD", "Cm"),
            "aileron": ("CD", "CY", "Cl", "Cn"),
            "rudder": ("CD", "CY", "Cl", "Cn"),
        }.items():
            for name in names:
                self.control[(surface, name)] = Grid2D(control_dir / f"{surface}_{name}.csv")

    def all_grids(self) -> Iterable[Grid2D]:
        yield from self.static.values()
        yield from self.control.values()

    @staticmethod
    def _copy(c: Coefficients) -> Coefficients:
        return Coefficients(**c.as_dict())

    def static_value(self, table: Grid2D, alpha_deg: float, beta_deg: float) -> float:
        if alpha_deg >= table.rows[0]:
            return table.interpolate(alpha_deg, abs(beta_deg))
        v0 = table.interpolate(table.rows[0], abs(beta_deg))
        v1 = table.interpolate(table.rows[0] + 2.0, abs(beta_deg))
        return v0 + (alpha_deg - table.rows[0]) * (v1 - v0) / 2.0

    def viterna(self, alpha_deg: float, beta_deg: float, want_lift: bool) -> float:
        lift = self.static["CL"]
        drag = self.static["CD"]
        anchor = 20.0 if alpha_deg > 0.0 else -12.0
        a0 = anchor * DEG_TO_RAD
        a = clamp(alpha_deg, -89.0, 89.0) * DEG_TO_RAD
        cls = self.static_value(lift, anchor, beta_deg)
        cds = self.static_value(drag, anchor, beta_deg)
        cdmax = 1.11 + 0.018 * self.SPAN_M**2 / self.AREA_M2
        b2 = (cds - cdmax * math.sin(a0) ** 2) / math.cos(a0)
        a1 = 0.5 * cdmax
        a2 = (
            (cls - cdmax * math.sin(a0) * math.cos(a0))
            * math.sin(a0) / math.cos(a0) ** 2
        )
        if want_lift:
            return a1 * math.sin(2.0 * a) + a2 * math.cos(a) ** 2 / math.sin(a)
        return cdmax * math.sin(a) ** 2 + b2 * math.cos(a)

    def static_coefficients(self, alpha_deg: float, beta_deg: float) -> Coefficients:
        abs_beta = clamp(abs(beta_deg), 0.0, 16.0)
        sign = -1.0 if beta_deg < 0.0 else 1.0
        c = Coefficients()
        if alpha_deg > 20.0 or alpha_deg < -12.0:
            c.CL = self.viterna(alpha_deg, abs_beta, True)
            c.CD = max(0.0, self.viterna(alpha_deg, abs_beta, False))
            anchor = 20.0 if alpha_deg > 0.0 else -12.0
            fade = 1.0 - smoothstep(abs(anchor), 90.0, abs(alpha_deg))
            c.Cm = self.static_value(self.static["Cm"], anchor, abs_beta) * fade
        else:
            c.CL = self.static_value(self.static["CL"], alpha_deg, abs_beta)
            c.CD = max(0.0, self.static_value(self.static["CD"], alpha_deg, abs_beta))
            c.Cm = self.static_value(self.static["Cm"], alpha_deg, abs_beta)
        lateral_fade = 1.0 - smoothstep(16.0, 90.0, abs(alpha_deg))
        c.CY = sign * self.static_value(self.static["CY"], alpha_deg, abs_beta) * lateral_fade
        c.Cl = sign * self.static_value(self.static["Cl"], alpha_deg, abs_beta) * lateral_fade
        c.Cn = sign * self.static_value(self.static["Cn"], alpha_deg, abs_beta) * lateral_fade
        return c

    def control_contributions(
        self,
        alpha_deg: float,
        beta_deg: float,
        delta_a_deg: float = 0.0,
        delta_e_deg: float = 0.0,
        delta_r_deg: float = 0.0,
        delta_c_deg: float = 0.0,
    ) -> Dict[str, Coefficients]:
        aileron = Coefficients()
        aileron_fade = 1.0 - smoothstep(12.0, 20.0, abs(alpha_deg))
        da_lookup = clamp(delta_a_deg, -10.0, 10.0)
        for name in ("CD", "CY", "Cl", "Cn"):
            value = self.control[("aileron", name)].interpolate(alpha_deg, da_lookup)
            setattr(aileron, name, value * delta_a_deg * aileron_fade)

        elevator = Coefficients()
        de_lookup = clamp(delta_e_deg, -10.0, 20.0)
        for name in ("CL", "CD", "Cm"):
            value = self.control[("elevator", name)].interpolate(alpha_deg, de_lookup)
            setattr(elevator, name, value * delta_e_deg)

        rudder = Coefficients()
        reflected_beta = -beta_deg if delta_r_deg < 0.0 else beta_deg
        rudder.CD = (
            (-1.0 if delta_r_deg < 0.0 else 1.0)
            * self.control[("rudder", "CD")].interpolate(alpha_deg, reflected_beta)
            * delta_r_deg
        )
        for name in ("CY", "Cl", "Cn"):
            value = self.control[("rudder", name)].interpolate(alpha_deg, reflected_beta)
            setattr(rudder, name, value * delta_r_deg)

        canard = Coefficients()
        dc_effective = -4.0 if delta_c_deg < -4.0 else clamp(delta_c_deg, -4.0, 15.0)
        dc_lookup = clamp(dc_effective, -4.0, 8.0)
        canard_fade = 1.0 - smoothstep(12.0, 16.0, abs(alpha_deg + dc_effective))
        for name in ("CL", "CD", "Cm"):
            value = self.control[("canard", name)].interpolate(alpha_deg, dc_lookup)
            setattr(canard, name, value * dc_effective * canard_fade)

        return {"aileron": aileron, "elevator": elevator, "rudder": rudder, "canard": canard}

    def coefficients(
        self,
        alpha_deg: float,
        beta_deg: float,
        speed_mps: float,
        delta_a_deg: float = 0.0,
        delta_e_deg: float = 0.0,
        delta_r_deg: float = 0.0,
        delta_c_deg: float = 0.0,
        p_rad_s: float = 0.0,
        q_rad_s: float = 0.0,
        r_rad_s: float = 0.0,
        alpha_dot_rad_s: float = 0.0,
        beta_dot_rad_s: float = 0.0,
    ) -> Tuple[Coefficients, Dict[str, Coefficients]]:
        c = self.static_coefficients(alpha_deg, beta_deg)
        controls = self.control_contributions(
            alpha_deg, beta_deg, delta_a_deg, delta_e_deg, delta_r_deg, delta_c_deg
        )
        for contribution in controls.values():
            c += contribution

        rate_blend = smoothstep(3.0, 5.0, speed_mps)
        inv2v = 0.5 / max(speed_mps, 5.0)
        c.CL += rate_blend * 5.62 * q_rad_s * self.MAC_M * inv2v
        c.CY += rate_blend * (
            -0.15 * p_rad_s * self.SPAN_M * inv2v
            + 0.34 * r_rad_s * self.SPAN_M * inv2v
        )
        c.Cl += rate_blend * (
            -0.33 * p_rad_s * self.SPAN_M * inv2v
            + 0.10 * r_rad_s * self.SPAN_M * inv2v
        )
        c.Cm += rate_blend * (
            -7.0 * q_rad_s * self.MAC_M * inv2v
            - 0.33 * alpha_dot_rad_s * self.MAC_M * inv2v
        )
        c.Cn += rate_blend * (
            -0.05 * p_rad_s * self.SPAN_M * inv2v
            - 0.08 * r_rad_s * self.SPAN_M * inv2v
            + 0.14 * beta_dot_rad_s * self.SPAN_M * inv2v
        )
        return c, controls

    def evaluate(
        self,
        alpha_deg: float,
        beta_deg: float,
        speed_mps: float,
        altitude_m: float = 0.0,
        **kwargs: float,
    ) -> AeroResult:
        c, controls = self.coefficients(alpha_deg, beta_deg, speed_mps, **kwargs)
        rho = isa_density(altitude_m)
        qbar = 0.5 * rho * speed_mps**2
        alpha = alpha_deg * DEG_TO_RAD
        beta = beta_deg * DEG_TO_RAD
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        ex = (ca * cb, sb, sa * cb)
        ey = (-ca * sb, cb, -sa * sb)
        ez = (-sa, 0.0, ca)
        force_frd = tuple(
            (-c.CD * qbar * self.AREA_M2) * ex[i]
            + (c.CY * qbar * self.AREA_M2) * ey[i]
            + (-c.CL * qbar * self.AREA_M2) * ez[i]
            for i in range(3)
        )
        moment_frd = (
            c.Cl * qbar * self.AREA_M2 * self.SPAN_M,
            c.Cm * qbar * self.AREA_M2 * self.MAC_M,
            c.Cn * qbar * self.AREA_M2 * self.SPAN_M,
        )
        force_gz = (force_frd[0], -force_frd[1], -force_frd[2])
        moment_gz = (moment_frd[0], -moment_frd[1], -moment_frd[2])
        return AeroResult(c, controls, force_frd, moment_frd, force_gz, moment_gz, rho, qbar)

    def solve_longitudinal_trim(
        self,
        speed_mps: float,
        delta_c_deg: float = 4.0,
        mass_kg: float = 150.0,
        altitude_m: float = 0.0,
        initial: Tuple[float, float] = (4.0, 1.0),
    ) -> Dict[str, float]:
        target_cl = mass_kg * 9.80665 / (
            0.5 * isa_density(altitude_m) * speed_mps**2 * self.AREA_M2
        )
        alpha, elevator = initial

        def residual(a: float, e: float) -> Tuple[float, float, Coefficients]:
            c, _ = self.coefficients(a, 0.0, speed_mps, delta_e_deg=e, delta_c_deg=delta_c_deg)
            return c.CL - target_cl, c.Cm, c

        for _ in range(20):
            f_lift, f_pitch, c = residual(alpha, elevator)
            if max(abs(f_lift), abs(f_pitch)) < 1e-11:
                break
            step = 1e-4
            fa = residual(alpha + step, elevator)
            fe = residual(alpha, elevator + step)
            j00 = (fa[0] - f_lift) / step
            j10 = (fa[1] - f_pitch) / step
            j01 = (fe[0] - f_lift) / step
            j11 = (fe[1] - f_pitch) / step
            determinant = j00 * j11 - j01 * j10
            if abs(determinant) < 1e-12:
                raise RuntimeError(f"singular trim Jacobian at {speed_mps:g} m/s")
            delta_alpha = (-f_lift * j11 + j01 * f_pitch) / determinant
            delta_elevator = (-j00 * f_pitch + j10 * f_lift) / determinant
            alpha += delta_alpha
            elevator += delta_elevator
        else:
            raise RuntimeError(f"trim did not converge at {speed_mps:g} m/s")

        f_lift, f_pitch, c = residual(alpha, elevator)
        return {
            "speed_mps": speed_mps,
            "alpha_deg": alpha,
            "elevator_deg": elevator,
            "canard_deg": delta_c_deg,
            "target_CL": target_cl,
            "CL": c.CL,
            "CD": c.CD,
            "Cm": c.Cm,
            "lift_residual": f_lift,
            "pitch_residual": f_pitch,
        }


def joint_angles_to_document_deflections(theta_deg: Iterable[float]) -> Tuple[float, float, float, float]:
    theta = tuple(theta_deg)
    if len(theta) != 8:
        raise ValueError("exactly eight aerodynamic joint angles are required")
    return (
        0.5 * (-theta[0] + theta[1]),
        0.5 * (theta[2] + theta[3]),
        0.5 * (theta[4] + theta[5]),
        0.5 * (theta[6] + theta[7]),
    )
