#!/usr/bin/env python3
"""Quick consistency checks for Honghu V5 aerodynamic CSV tables."""

from __future__ import annotations

import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TABLE_DIR = ROOT / "simulation_models/models/honghu_wing_150kg_v5/aero_tables"
EXPECTED = {
    "CL": {(-2.0, 0.0): 0.1388, (0.0, 0.0): 0.3012, (16.0, 16.0): 1.1699},
    "CD": {(-2.0, 0.0): 0.0259, (0.0, 0.0): 0.0293, (16.0, 16.0): 0.2499},
    "CY": {(-2.0, 2.0): -0.0326, (0.0, 0.0): 0.0, (16.0, 16.0): -0.1979},
    "Cm": {(-2.0, 0.0): -0.0488, (0.0, 0.0): -0.0534, (16.0, 16.0): -0.0608},
    "Cl": {(-2.0, 16.0): 0.0048, (0.0, 0.0): 0.0, (16.0, 16.0): -0.0258},
    "Cn": {(-2.0, 16.0): 0.0212, (0.0, 0.0): 0.0, (16.0, 16.0): 0.0166},
}


def load_table(name: str):
    path = TABLE_DIR / f"{name}.csv"
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        filtered = (line for line in f if line.strip() and not line.startswith("#"))
        reader = csv.reader(filtered)
        header = next(reader)
        beta = [float(x) for x in header[1:]]
        for row in reader:
            rows.append((float(row[0]), [float(x) for x in row[1:]]))
    return beta, rows


def main() -> int:
    for name, expected_points in EXPECTED.items():
        beta, rows = load_table(name)
        lookup = {(alpha, beta_col): value for alpha, values in rows for beta_col, value in zip(beta, values)}
        if len(rows) != 10 or len(beta) != 9:
            raise SystemExit(f"{name}: unexpected shape rows={len(rows)} beta={len(beta)}")
        for key, expected in expected_points.items():
            actual = lookup[key]
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6):
                raise SystemExit(f"{name}{key}: expected {expected}, got {actual}")
        print(f"{name}: OK ({len(rows)}x{len(beta)})")
    print("Honghu V5 aero tables: all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
