# Honghu Wing A1 Gazebo V8

V8 is an independent PX4 / Gazebo model built around the A1 V2.5(2) data. It
reuses the 3.96 m visual meshes and PX4 actuator topology, but it does not reuse
V7 aerodynamic equations. The current phase-1 ground model deliberately uses
rigid wheel carriers; suspension is deferred until the rigid-contact baseline is
closed.

## Run and verify

```sh
make px4_sitl gz_honghu_wing_150kg_v8
python3 Tools/honghu/check_honghu_v8.py
gz sdf -k simulation_models/models/honghu_wing_150kg_v8/model.sdf
```

`PX4_GZ_MAX_STEP_SIZE=0.002` is set by airframe 4028. This 500 Hz value is the
repeatably accepted real-time takeoff/route baseline. Sub-2 ms physics currently
overruns the SITL bridge with clock/pose traffic and must not be described as a
converged production setting until that capacity limit is removed. The launch script sends a
complete Gazebo physics request, including gravity and magnetic field; sending
only `max_step_size` disables parts of physics because `gz.msgs.Physics` is not
a patch message.

The V8 WGS84 origin is `28.5712315 deg N, 121.5759172 deg E, 0 m`. Airframe
4028 also sends these coordinates through Gazebo's run-time
`set_spherical_coordinates` service. This is deliberate: `px4-rc.gzsim` reuses
an already-running Gazebo server, whose old spherical coordinates would
otherwise survive an SDF edit.

To upload the current QGC XY plan without arming the vehicle:

```sh
python3 Tools/honghu/upload_qgc_plan.py \
  /home/fly/px4_reference_docs/current/模仿XY航线规划.plan
```

The helper uses PX4's local Onboard MAVLink receive endpoint at UDP 14540,
clears the old vehicle mission, uploads every `SimpleItem`, and waits for the
mission ACK. QGC can alternatively load the same `.plan` and upload it after
the vehicle reports `Ready for takeoff!`.

## Coordinate and sign contract

- PDF / PX4 body axes are FRD: X forward, Y right, Z down.
- Gazebo `base_link` is FLU at assembled CG: X forward, Y left, Z up.
- Vector conversion is `R = diag(1,-1,-1)` for both forces and moments.
- The CAD/nose-origin visual meshes are translated `+1.57 m` in model X.
- PDF points become `p_GZ = (x_PDF+1.57, -y_PDF, -z_PDF)`.

The aerodynamic moment is first formed in FRD,
`M_FRD=qS[b Cl, c Cm, b Cn]`, then converted once to
`M_GZ=(Mx_FRD,-My_FRD,-Mz_FRD)`.

Positive Gazebo joint angles mean: aileron/elevator trailing edge up, rudder
trailing edge to aircraft right, and canard trailing edge down. Joint angles and
PDF virtual deflections are both reported in degrees:

```text
delta_a = 0.5*(-theta_left_aileron + theta_right_aileron)
delta_e = 0.5*( theta_left_elevator + theta_right_elevator)
delta_r = 0.5*( theta_left_rudder   + theta_right_rudder)
delta_c = 0.5*( theta_left_canard   + theta_right_canard)
```

Thus positive `delta_a/e/r/c` produces positive FRD `Cl/Cm/Cn/Cm` respectively;
in Gazebo those principal moments are `+Mx/-My/-Mz/-My`.

## Physical model

The assembled targets are `m=150 kg`, `S=2.42 m2`, `b=3.96 m`, `c=0.62 m`.
The generator subtracts every child link through the parallel-axis theorem so
the assembled CG and FLU inertia remain exactly at the PDF targets.

The phase-1 gear has no prismatic joints:

- main wheels: direct revolute joints on `base_link`, free spin about local Y;
- nose carrier: direct steering joint on `base_link`, plus a free wheel-spin joint;
- wheel locations: `(-0.291274, +/-0.524303, -0.4551) m` and
  `(0.924852, 0, -0.4706) m`;
- contact: main-wheel `mu/mu2=0.8/2.0`, nose-wheel `mu/mu2=1.2/3.0`; all three use `kp=2e6 N/m`, `kd=2e4 N s/m`, `max_vel=0.2 m/s`, and `min_depth=0.5 mm`.

This design separates wheel/ground contact and steering from suspension
numerics. A later suspension revision should be a separate, measurable change,
not an implicit modification of this baseline.

Static aerodynamic tables are interpolated in alpha/beta with documented
symmetry. Control-table values are local derivatives per degree and are
multiplied by signed `delta_doc`; zero deflection therefore gives exactly zero
control contribution. Dynamic derivatives use `p,q,r,alpha_dot,beta_dot`.
Positive post-stall alpha uses a continuous Viterna extension; beta is clamped at
16 degrees with a flag. The engine plugin is the only thrust source. It
subscribes to the PX4 Gazebo ESC bridge topic
`/model/<runtime-model>/honghu_v8/motor_command` mirror and interpolates the PDF propulsion and
fuel data using a 0..1000 ESC command.
The front-view counter-clockwise propeller rotates about Gazebo `+X`; its
signed shaft torque is applied to the airframe with the opposite `-X` sign.
The thrust axis is pitched 3 deg downward in aircraft coordinates. In Gazebo
FLU this is `d_GZ=[cos(3 deg),0,-sin(3 deg)]`; the force is applied at
`(-1.23,0,+0.12) m`, so the plugin includes both the vertical thrust component
and the pitch moment about the CG rather than treating thrust as CG-aligned.

## Diagnostics

All topics use the runtime model instance name:

```text
/model/<name>/honghu_v8/aero_state
/model/<name>/honghu_v8/force_frd
/model/<name>/honghu_v8/moment_frd
/model/<name>/honghu_v8/force_gz_flu
/model/<name>/honghu_v8/moment_gz_flu
/model/<name>/honghu_v8/propulsion_state
```

`aero_state` (`gz.msgs.Double_V`) index map:

| Indices | Meaning |
|---|---|
| 0..3 | speed m/s, alpha deg, beta deg, density kg/m3 |
| 4..5 | alpha_dot, beta_dot rad/s |
| 6..8 | FRD p, q, r rad/s |
| 9..14 | total CL, CD, CY, Cl, Cm, Cn |
| 15..22 | theta_joint[0..7] deg |
| 23..26 | delta_a, delta_e, delta_r, delta_c deg |
| 27..50 | aileron, elevator, rudder, canard contributions; each CL,CD,CY,Cl,Cm,Cn |
| 51..74 | eight joint axes in base_link, XYZ per axis |
| 75 | flags: beta clamp=1, post-stall=2, control extrapolation=4, low speed=8, control-source clamp=16, derived static data=32 |

`propulsion_state` indices are target throttle, filtered throttle, altitude,
airspeed, RPM, thrust N, torque N m, fuel kg/h and flags (input clamp=1,
fuel-table clamp=2).

## Current verification boundary

- SDF validation, the static V8 contract, the full PX4 SITL build, target startup,
  coordinate/moment signs, and 917 aerodynamic table/sign/continuity/trim checks pass.
- Airframe 4028 uses an 8 deg positive pitch-setpoint limit, a 6 deg/s positive
  pitch-rate limit, `FW_PR_FF_RWTO=6.6`, `FW_PR_RWTO_Q=2.0`, and
  `FW_PR_P/I/D/FF=0.40/0.04/0.10/0.75`. Both the pitch-rate integrator and the
  TECS pitch integrator (`FW_T_I_GAIN_PIT=0.05`) remain enabled to absorb
  unknown real trim. The pitch-rate integrator is bounded by `FW_PR_IMAX=0.12`.
  The high runway feed-forward is removed over the first 2 deg/s of measured
  nose-up rate; it is not retained into the climb.
- The bounded airborne route-following baseline is
  `FW_RR_P/I/D/FF=0.26/0.05/0.06/1.45`, `FW_R_TC=0.65 s`,
  `FW_R_RMAX=20 deg/s`, `FW_R_LIM=30 deg`, `NPFG_PERIOD=20 s`, and
  `NPFG_SW_DST_MLT=0.32`, with `FW_T_RLL2THR=20`. Runway steering retains the separate
  `RWTO_NPFG_PERIOD=8 s` setting.
- `NAV_ACC_RAD=250 m` is the V8 default fixed-wing waypoint acceptance radius.
  A QGC waypoint acceptance value of zero falls back to this lower bound. The
  fixed-wing controller also computes an NPFG switch distance as the track-error
  bound times `NPFG_SW_DST_MLT`. With the selected 20 s period, 0.32 gives about
  206..208 m in the clean QGC logs. An earlier 10 s-period probe raised the
  multiplier to 0.90 and did make segment changes occur at 270..308 m, but the
  controller then captured the backward extension of the next straight leg; it
  did not construct a tangent circular fillet. That 2 ms probe completed all
  waypoints but produced large lateral transients and dipped to 32.63 m, so 0.90
  was rejected. Exact constant-radius fly-by turns require mission fillet/arc
  geometry rather than a larger acceptance or switch distance alone.
- The final default 2 ms takeoff report is
  `analysis_outputs/honghu_v8_takeoff_final_defaults_2ms.json`: rotation at
  43.86 m/s, sustained liftoff at 44.12 m/s, Gazebo-truth maximum absolute pitch
  8.16 deg, runway cross-track maximum 0.038 m, and final climb 47.1 m.
- The most recent dynamically passed no-foldback five-leg route report is
  `analysis_outputs/honghu_v8_route_final_defaults_2ms.json`. It passes all
  checks: stable cross-track RMS/p95 are 1.80/3.68 m, altitude is
  35.19..53.92 m, and airspeed is 39.00..45.92 m/s. Every settled leg has RMS
  below 3 m. The canards remain near +3.99 deg and do not enter pitch allocation.
  The separate rejected 0.90 probe is retained as
  `analysis_outputs/honghu_v8_route_npfg_sw090_2ms.json`.
- Two clean QGC mission logs with all inner-loop gains unchanged compared
  `NPFG_PERIOD=15 s` and `20 s`. The 20 s case reduced roll saturation and
  height/airspeed excursions while producing the desired smooth, arc-like
  NPFG line capture. It is now the V8 default. This is a guidance transient,
  not an explicit constant-radius geometric fillet.
- `RWTO_DIR_MIN=50 m` rejects a TAKEOFF direction derived from a mission item
  colocated with the launch point. Such an item still defines clearance
  altitude, while the next valid mission waypoint defines runway/climbout
  direction. If the TAKEOFF item is farther than 50 m, its own direction is
  retained. The production QGC plan uses the Gazebo origin as planned home and
  a TAKEOFF point 1.893 km away on true bearing 24.97 deg.
- The targeted colocated-TAKEOFF regression is
  `analysis_outputs/honghu_v8_takeoff_coincident_npfg20_2ms.json`. It passed by
  tracking the next eastbound waypoint: runway cross-track maximum 0.028 m,
  liftoff at 44.31 m/s, and maximum takeoff pitch 8.26 deg.
- Two repeated user QGC runs of the corrected 20-item production mission are
  `log/2026-07-16/13_08_15.ulg` and `13_32_26.ulg`. Both loaded
  `NPFG_PERIOD=20 s`, `NAV_ACC_RAD=250 m`, and `RWTO_DIR_MIN=50 m` without
  in-flight parameter changes, had no ULog dropouts or flight failsafe, started
  at mission item 0, and reached the landing item 18. Their runway-line
  cross-track maxima were both 0.455 m; airborne roll tracking RMS was
  1.44/1.43 deg. Most settled straight legs were within 0.2..7.7 m RMS, while
  the near-89-degree corner retained a repeatable approximately 150 m peak cut.
  The runs ended during the landing item and do not validate touchdown.
- The final continuous-loiter report is
  `analysis_outputs/honghu_v8_flight_final_defaults_2ms.json`. It passes with
  24.95 s settled observation, 39.22..48.65 m altitude, 39.03..42.78 m/s
  airspeed, 31.45 deg maximum roll, and 6.50 deg maximum pitch.
- The route test uses 1200 m legs and 30 deg course changes. PX4 takeoff tracks
  clearance altitude plus a fixed 10 m buffer, so the test uses a 40 m takeoff
  clearance before 50 m waypoints to avoid an artificial 60 -> 50 m transition.
- Liftoff reporting uses a 0.5 m candidate crossing that is accepted only after
  the model subsequently clears 2 m. This prevents a nose-wheel unload or a
  short hop from being reported as a completed takeoff.
- The earlier static 60 s and 8 m/s taxi campaigns passed. Apparent sub-2 ms
  takeoff sensitivity was traced primarily to PX4/Gazebo state backlog rather
  than the rigid rolling gear. The 2 ms bridge-synchronized result is the
  production requirement.
- Full step-size convergence, automatic landing, and the -50 deg canard braking
  aerodynamics are not yet accepted. The current world ground is 30000 x 30000 m;
  leaving that finite surface must not be interpreted as a landing-gear failure.
- `模仿XY航线规划.plan` contains 21 mission items and is accepted by PX4 with
  the V8 origin, but the entire approximately 12 km-radius route and its
  automatic landing have not yet completed a dynamic acceptance flight.
