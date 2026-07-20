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

The private GitHub destination is
`git@github.com:EIHEIK/PX4-Autopilot-NewAero.git`. The initial exported
baseline is commit `51e344baa19ecc398bc4f9388d53772d6b228bea`; local and remote
`main` were verified to match after the first push.

The first V8 run from the new workspace completed successfully on 2026-07-20.
Because `build/` is intentionally excluded, the new SITL rootfs had no prior
`parameters.bson`; PX4 therefore performed one normal airframe autoconfiguration
for `SYS_AUTOSTART=4028` and printed the full `curr -> new` parameter list.
The resulting ULog reports `SYS_AUTOCONFIG=0`, and parameter plus backup BSON
files were created. Parameters written with `param set` in airframe 4028 are
intentionally forced to the validated baseline on every boot; parameters using
`param set-default` may retain saved QGC overrides.
