# PX4-Autopilot-NewAero GitHub export notes

This repository was exported from the working tree at
`/home/fly/PX4-Autopilot-canard-2026.6.2` on 2026-07-20.

The export preserves the PX4 Git history and the complete Honghu V5-V8 source
tree, but intentionally excludes local build products, raw ULogs, temporary
PDF renderings, generated diagnostic images and the large tuning-probe archive.
Four accepted V8 JSON reports are retained under `analysis_outputs/`.

`Tools/simulation/gz` is vendored in this repository instead of remaining a
Git submodule. The Honghu V8 world and the project-specific default-world
change were uncommitted modifications of the upstream PX4 Gazebo-models
submodule. Vendoring the directory ensures a normal clone of this single
repository contains the exact Gazebo world required by V8.

The source repository was not modified by this export.
