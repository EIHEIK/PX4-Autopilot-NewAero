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

The V8 magnetic vector is the PX4 WMM-2020 value at the configured origin,
converted from NED to Gazebo ENU:
`[-3.55621019e-6, 3.46940371e-5, -3.25102706e-5] T`. Airframe 4028 exports the
same vector so the runtime physics request cannot silently replace it with the
generic Gazebo default.

Gazebo Harmonic also regenerates its native magnetometer field from a coarse
table and exposes it through a historical NED / left-handed convention. That
path is not a valid three-dimensional rotation when an ENU/FLU pose is tilted.
V8 therefore removes only its native magnetometer sensor and uses
`HonghuMagnetometerV8`: the plugin rotates the exact NED field into body FRD and
publishes the inverse legacy representation expected by PX4's unchanged
`[-Y,-X,+Z]` Harmonic callback. The earlier V8-only two-dimensional declination
parameter has been removed; official and older models retain their original
bridge and sensor behavior.

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

The mass model is compositional rather than a monolithic `base_link` inertia:

- the physical baseline is the Word table-3 `73 kg` aircraft with full internal
  fuel, including its supplied complete inertia tensor;
- its current longitudinal CG is the explicit engineering assumption
  `x=-1.58 m`, 10 mm aft of the common target/reference at `x=-1.57 m`;
- the Word `CGz=-0.03 m` is retained;
- movable surfaces and wheels remain explicit child links, so their properties
  are subtracted from the residual `base_link` and are never counted twice;
- an internal fixed link named `adjustable_ballast` closes the requested total
  mass and CG through the parallel-axis theorem.

The production 150 kg model uses a `77 kg` ballast centred at Gazebo FLU
`(0.009480519, 0, -0.028441558) m`. Its intrinsic tensor is solved so the full
assembly exactly reproduces the Word 150 kg inertia
`Ixx/Iyy/Izz=25.86/39.14/59.12 kg m2` and the supplied products of inertia.
The isolated 100 kg derivative uses `27 kg` at
`(0.027037037, 0, -0.081111111) m`. It retains the same ballast geometry
(constant intrinsic inertia per unit mass), giving the physically composed
100 kg FLU tensor:

```text
[25.714298926   0.019597403   2.983554188]
[ 0.019597403  33.951414391  -0.000796104]
[ 2.983554188  -0.000796104  53.856336244] kg m2
```

Directly interpolating the complete 73/150 kg tensors was rejected for this
architecture: together with the supplied 73 kg vertical CG it would require a
27 kg ballast with a negative principal inertia. The generator constants
`BASE_73_CG_PDF_FRD` and `TARGET_ASSEMBLED_CG_PDF_FRD` are the intended
physical-CG inputs; changing them recomputes both ballast positions rather than
editing SDF inertial blocks by hand. `REFERENCE_CG_PDF_FRD` remains the fixed
geometry and aerodynamic moment reference unless a deliberate coordinate-system
migration is performed.

The aerodynamic geometry remains `S=2.42 m2`, `b=3.96 m`, `c=0.62 m`.

The phase-1 gear has no prismatic joints:

- main wheels: direct revolute joints on `base_link`, free spin about local Y;
- nose carrier: direct steering joint on `base_link`, plus a free wheel-spin joint;
- wheel locations: `(-0.291274, +/-0.524303, -0.4551) m` and
  `(0.924852, 0, -0.4706) m`;
- contact: main-wheel `mu/mu2=0.8/2.0`, nose-wheel `mu/mu2=1.0/1.0`; all three use `kp=2e6 N/m`, `kd=2e4 N s/m`, `max_vel=0.2 m/s`, and `min_depth=0.5 mm`;
- nose steering joint controller: `P/I/D=500/200/1`, integral limit `40 N m`, command limit `100 N m`.

The earlier `1000/500/30` steering controller was rejected. It could force the
loaded tyre to its static target angle, but excited the rigid contact/joint
constraint and generated lateral IMU oscillations up to about `1.9 m/s2` that
were absent from pose-derived acceleration. The current controller preserves a
visible heading correction while keeping that discrepancy below about
`0.05 m/s2` in the 20-degree disturbed taxi diagnostic.

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
| 76..77 | source simulation time us, diagnostic sequence |

`propulsion_state` indices 0..8 are target throttle, filtered throttle,
altitude, airspeed, RPM, thrust N, torque N m, fuel kg/h and flags (input
clamp=1, fuel-table clamp=2). Indices 9..10 are source simulation time us and
diagnostic sequence.

For V8 instances, `GZBridge` also converts these two diagnostic vectors to the
uORB topics `honghu_v8_aero_state` and `honghu_v8_propulsion_state`. The logger
records them with a 20 ms minimum interval (50 Hz). This retains actual Gazebo joint
feedback and the engine lag state in ULog for offline rigid-body coefficient
reconstruction without starting external topic readers during flight. Run:

```bash
python3 Tools/honghu/analyze_honghu_v8_aero_coefficients.py <flight.ulg>
```

The analyzer restores `estimator_sensor_bias.accel_bias` to PX4's filtered
`vehicle_acceleration` before applying the force balance. PX4 has already
removed that EKF estimate from the published controller signal; using it
directly creates false CD/CY residuals. This is an offline-only correction and
does not change the estimator, controller, or simulator at runtime.

The implementation and the selected standard-mission result are documented in
`Documentation/honghu/HONGHU_V8_AERO_COEFFICIENT_VALIDATION_2026-07-21.md`.

## Current verification boundary

- The 2026-07-21 standard 20-item `模仿XY航线规划.plan` truth ULog provides
  715.348 s and 14,308 valid airborne coefficient-reconstruction samples with
  actual Gazebo joint feedback. Filtered inversion/independent-model
  correlations are 0.999877/0.999872/0.999547/0.999400/0.989857/0.993070 for
  CL/CD/CY/Cl/Cm/Cn. CD bias is only +0.00000164 after restoring the EKF
  acceleration-bias estimate. This validates software and rigid-body closure,
  not real-aircraft accuracy.
- The corresponding complete-mission report is
  `analysis_outputs/honghu_v8_standard_plan_offline_diagnostics_2ms.json`:
  rotation 43.914 m/s, liftoff 44.163 m/s, maximum takeoff truth pitch
  8.227 deg, runway-frame cross-track 0.144 m, and mission progression to LAND
  item 18. The run stops near 5 m AGL, so touchdown and rollout remain outside
  this acceptance.
- A/B tests traced the two 2026-07-21 high-speed pitch divergences to six
  concurrent external `gz topic --json-output` observers, not to the uORB truth
  messages, aerodynamic signs, or nose-wheel parameters. Dynamic acceptance
  now reads MAVLink during flight and loads Gazebo truth from ULog only after
  shutdown.
- The new 2 ms offline-diagnostic regression passes: rotation 44.028 m/s,
  liftoff 44.208 m/s, maximum truth pitch 8.255 deg, ground cross-track
  0.269 m, and final truth climb 46.721 m. The ULog contains 2,030 contiguous
  50 Hz aerodynamic samples with actual joint feedback.

- SDF validation, the static V8 contract, the full PX4 SITL build, target startup,
  coordinate/moment signs, and 917 aerodynamic table/sign/continuity/trim checks pass.
- Airframe 4028 now separates longitudinal authority by flight phase. Cruise
  uses a 10 deg positive pitch-setpoint limit and 10 deg/s positive pitch-rate
  limit; runway takeoff and low-height landing retain 8 deg and 6 deg/s. The
  altitude is now only a transition trigger: climbing through 50 m above the
  takeoff point releases takeoff limits over a 5 s smoothstep, and the first
  descent below 50 m AGL in a LAND item introduces landing limits over the same
  5 s duration. The resulting weights no longer follow each subsequent height
  sample. Rate-limit changes also retain a 2 deg/s^2 safety slew.
  `RWTO_ROT_TIME=6 s`,
  `FW_PR_FF_RWTO=6.6`, `FW_PR_FF_LND=6.6`, `FW_PR_RWTO_Q=2.0`, and
  `FW_PR_P/I/D/FF=0.40/0.04/0.10/0.90`. The V8-only
  fixed canard command is +6 deg (`FW_CANARD_TO=0.4`) and the matching nominal
  pitch trim is `TRIM_PITCH=-0.02`. Both the pitch-rate integrator and the
  TECS pitch integrator (`FW_T_I_GAIN_PIT=0.05`) remain enabled to absorb
  unknown real trim. The pitch-rate integrator is bounded by `FW_PR_IMAX=0.12`.
  The high runway feed-forward is removed over the first 2 deg/s of measured
  nose-up rate and is not retained into the climb. Landing flare has an
  independent feed-forward path that is enabled only by the LANDING phase and
  the final flare/wheel-control window; it no longer borrows takeoff logic.
- The rear propeller geometry has a 10 deg ground-strike angle. The automated
  V8 takeoff guard is therefore 8.5 deg, rather than the former 12 deg generic
  attitude bound. The validated 2026-07-29 candidate reached 7.96 deg maximum
  Gazebo-truth pitch, lifted off at 42.85 m/s with 6.24 deg pitch, and retained
  about 2 deg geometric margin while completing the 30 deg loiter regression.
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
- Full step-size convergence, automatic landing quality, and the -50 deg canard
  braking aerodynamics are not yet accepted. The V8 world now uses a
  30000 x 30000 x 1 m solid collision box (top surface at z=0) below the
  unchanged 30000 x 30000 m visual plane. This avoids DART / FCL's bounded
  broadphase proxy for an SDF plane, which previously allowed the aircraft to
  fall through near Gazebo Y=1.1 km even though the plane was drawn to 30 km.
- A post-fix closed-loop takeoff-and-landing test at the former failure location reached the
  runway at 1.53 m/s sink rate and 34.13 m/s groundspeed, remained upright, and
  held the base-link height at 0.5145 m throughout 66.6 s of rollout. Ground
  penetration is therefore fixed; flare sink rate, braking, land detection,
  and the -50 deg canard airbrake remain separate landing-quality work.
- The current `模仿XY航线规划.plan` contains 20 mission items. The selected
  coefficient-validation log progressed from item 0 to landing item 18 without
  mission failure. Landing-item entry at about 5 m AGL is not an accepted
  automatic touchdown and rollout test.
