# Honghu V8 recent operations: takeoff envelope and landing canard

This repository note is a condensed mirror. The authoritative and more detailed record is:

```text
/home/fly/px4_reference_docs/current/HONGHU_V8_RECENT_OPERATIONS_2026-07-31.md
```

## Current repository boundary

```text
branch: main
HEAD/origin/main: 89c1008a5a
```

The source changes described below are currently local and uncommitted. In particular, the remote commit
does not yet contain the final 6/7-degree takeoff envelope or robust V8 touchdown detection.

## Remote canard-state-machine update

Commit `89c1008a5a`:

- removed the forced maximum nose-down elevator command after touchdown;
- added `FW_CANARD_RETR=0.5`, retracting the canard to neutral immediately after touchdown;
- changed the aerodynamic-brake delay to 5 s;
- retained canard types 19/20 outside pitch allocation;
- fixed a control-allocation compile type mismatch.

Follow-up review found and corrected locally:

- undefined `FW_TKO_P_RATE` removed;
- circular-landing normal-load sign changed to `-accel.z/g`, matching straight landing;
- the independent landing-test handoff waypoint changed from 50 to 40 m to prevent an unrelated altitude-acquisition orbit.

## Valid takeoff envelope

The remote combination `FW_TKO_PITCH_MIN=6°` and `RWTO_PMAX=4°` was internally inconsistent.
The accepted V8 settings are:

```text
FW_TKO_PITCH_MIN = 6 deg
RWTO_PMAX        = 7 deg
RWTO_ROT_AIRSPD  = 38 m/s
```

The static contract now rejects a minimum takeoff pitch above `RWTO_PMAX`.

The 150 kg takeoff regression achieved:

```text
liftoff airspeed         43.999 m/s
ground roll              537.38 m
maximum truth pitch        7.456 deg
canard                    about +6 deg
```

The no-foldback route completed all waypoints with stable-leg cross-track RMS 2.433 m, maximum roll
19.837°, roll-tracking RMS 1.601°, altitude 40.049–51.181 m and airspeed 39.403–41.372 m/s.

## Robust V8 touchdown detection

The classic-route log `2026-07-30/18_32_00.ulg` reached LAND and touched down, but the canard remained at
about +6°. The actuator bridge and Gazebo joint agreed, so the fault was before the phase transition.

The old no-rangefinder AGL fallback used `_current_altitude - _takeoff_ground_alt`. The takeoff reference
is reset continuously outside AUTO_TAKEOFF, so this was not a valid landing-height reference. The 1.5 g
touchdown pulse was also narrow enough to be missed by the landing-control sampling.

`FW_CANARD_LND_PK` was added with global default zero. This keeps legacy aircraft behavior unchanged.
V8 explicitly enables:

```text
FW_CANARD_LND_H   = 0.1 m
FW_CANARD_LND_NZ  = 1.3 g
FW_CANARD_LND_H2  = 1.0 m
FW_CANARD_LND_PK  = 0.2 s
```

With peak holding enabled, the no-rangefinder fallback uses the landing controller's
terrain/landing-surface-referenced `landing_height`. The load peak is held for 0.2 s and is accepted only
inside the actual near-ground gate. Straight and circular landing use the same logic and sign.

The automatic landing regression `2026-07-30/19_15_53.ulg` verified the actual Gazebo sequence:

```text
airborne            +6 deg
touchdown             0 deg
about 5.12 s later  -50 deg
```

The landing script still reports FAIL only because sink rate was 1.562 m/s, above its strict 1 m/s limit.
Canard timing, attitude, ground collision and final landed state passed.

## Latest user classic-route log

Log: `2026-07-30/19_22_25.ulg`

```text
LAND start                    259.372 s
first ground contact          284.208 s
canard neutral                284.332 s
neutral delay after contact     0.124 s
canard -50 deg                289.336 s
airbrake delay                  5.004 s
```

The elevator remained under the normal landing controllers. It was mostly +8 to +12° in the final flare,
briefly reached +24.29° for one 100 ms servo-log sample, and produced a -6.86° pulse for about 0.1 s at
contact before returning positive. Left/right elevator and canard commands were identical.

The 20 Hz `vehicle_acceleration` log recorded only 1.178 g maximum, while 250 Hz `sensor_combined`
captured a 22.43 g, approximately 8 ms raw impact. This is a rigid-wheel/contact numerical impulse, not
a validated structural load. It explains why low-rate logs can miss the trigger. Deploying the -50°
canard produced no second load or sustained pitch transient.

Local analysis:

```text
analysis_outputs/honghu_v8_pitch_envelope_6_7/
analysis_outputs/honghu_v8_canard_touchdown_fix/
analysis_outputs/honghu_v8_latest_landing_controls/
```

## Current verification and remaining work

Passed:

- PX4 SITL build;
- V8 static contract and 917 aerodynamic checks;
- legal takeoff pitch envelope;
- takeoff and no-foldback-route regressions;
- canard touchdown retraction and delayed aerodynamic braking;
- ground collision without fall-through.

Remaining:

- reduce touchdown sink rate below 1 m/s;
- replace rigid-contact impulses with a physically validated gear/load model if structural loads are needed;
- implement or validate realistic wheel braking and stopping distance;
- complete 0.5/1/2 ms convergence testing;
- restore consistent V8 truth-topic logging in full user missions;
- commit and push the current local changes.
