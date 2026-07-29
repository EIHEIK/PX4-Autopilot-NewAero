/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file runway_takeoff_params.c
 *
 * Parameters for runway takeoff
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

/**
 * Runway takeoff with landing gear
 *
 * @boolean
 * @group Runway Takeoff
 */
PARAM_DEFINE_INT32(RWTO_TKOFF, 0);

/**
 * Specifies which heading should be held during the runway takeoff ground roll.
 *
 * 0: airframe heading when takeoff is initiated
 * 1: position control along runway direction (bearing defined from vehicle position on takeoff initiation to MAV_CMD_TAKEOFF
 *    position defined by operator)
 *
 * @value 0 Airframe
 * @value 1 Runway
 * @min 0
 * @max 1
 * @group Runway Takeoff
 */
PARAM_DEFINE_INT32(RWTO_HDG, 0);

/**
 * Minimum distance used to define the mission takeoff direction
 *
 * If the distance from the runway start to MAV_CMD_NAV_TAKEOFF is smaller than
 * this value, the takeoff location is treated as a clearance-altitude marker
 * rather than a direction point. The direction to the next valid mission
 * waypoint is used instead, falling back to the airframe heading when no such
 * waypoint is available. Set to zero to retain the legacy behavior.
 *
 * @unit m
 * @min 0
 * @max 500
 * @decimal 1
 * @increment 1
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_DIR_MIN, 0.0f);

/**
 * Max throttle during runway takeoff.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_MAX_THR, 1.0);

/**
 * Pitch setpoint during taxi / before takeoff rotation airspeed is reached.
 *
 * A taildragger with steerable wheel might need to pitch up
 * a little to keep its wheel on the ground before airspeed
 * to takeoff is reached.
 *
 * @unit deg
 * @min -10.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_PSP, 0.0);

/**
 * Throttle ramp up time for runway takeoff
 *
 * @unit s
 * @min 1.0
 * @max 15.0
 * @decimal 2
 * @increment 0.1
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_RAMP_TIME, 2.0f);

/**
 * Ground taxi test mode during runway takeoff
 *
 * When enabled, the runway takeoff state machine stays in the ground-roll
 * phase and does not transition to climbout when rotation airspeed is reached.
 * This is intended for steerable nose-wheel tuning in simulation. Disable it
 * for normal takeoff tests.
 *
 * @boolean
 * @group Runway Takeoff
 */
PARAM_DEFINE_INT32(RWTO_TAXI_TEST, 0);

/**
 * Legacy taxi-to-takeoff switch
 *
 * This parameter is kept for compatibility with older Honghu test logs. Normal
 * runway takeoff no longer uses RWTO_TAXI_TEST / RWTO_TAXI_TOFF as a staged
 * taxi-to-takeoff mode. Keep RWTO_TAXI_TEST disabled for formal takeoff tests.
 *
 * @boolean
 * @group Runway Takeoff
 */
PARAM_DEFINE_INT32(RWTO_TAXI_TOFF, 0);

/**
 * Throttle used in runway taxi test mode
 *
 * This open-loop throttle replaces RWTO_MAX_THR while RWTO_TAXI_TEST is
 * enabled. Keep it low enough that the aircraft remains on the runway for
 * nose-wheel steering tuning.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_THR, 0.25f);

/**
 * Maximum airspeed allowed in runway taxi test mode
 *
 * If calibrated airspeed exceeds this value while RWTO_TAXI_TEST is enabled,
 * the open-loop taxi throttle is reduced to idle. Set to a conservative value
 * for low-speed ground steering tests.
 *
 * @unit m/s
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_ARSP, 15.0f);

/**
 * Maximum ground speed allowed in runway taxi test mode
 *
 * If horizontal ground speed exceeds this value while RWTO_TAXI_TEST is
 * enabled, the open-loop taxi throttle is reduced to idle. This protects
 * long ground steering tests from being interpreted as takeoff by the fixed
 * wing land detector.
 *
 * @unit m/s
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_GSPD, 4.0f);

/**
 * Target ground speed in runway taxi test mode
 *
 * This is the closed-loop ground speed target used only when RWTO_TAXI_TEST is
 * enabled. It is intentionally independent from takeoff airspeed.
 *
 * @unit m/s
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_SPD, 10.0f);

/**
 * Feed-forward throttle in runway taxi test mode
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_THR_FF, 0.22f);

/**
 * Minimum throttle in runway taxi test mode
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_TMIN, 0.0f);

/**
 * Maximum throttle in runway taxi test mode
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_TMAX, 0.35f);

/**
 * Ground speed proportional gain in runway taxi test mode
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_SPD_P, 0.025f);

/**
 * Ground speed integral gain in runway taxi test mode
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.001
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_SPD_I, 0.003f);

/**
 * Cross-track heading correction gain in runway taxi test mode
 *
 * Converts lateral line error to a small heading correction while taxiing.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_XTK_P, 0.015f);

/**
 * Acceptance radius for runway taxi mission waypoints
 *
 * Used by the independent RWTO_TAXI_TEST route state machine when a mission
 * item does not define a valid acceptance radius.
 *
 * @unit m
 * @min 1.0
 * @decimal 1
 * @increment 1.0
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_ACC, 12.0f);

/**
 * Maximum cross-track heading correction in runway taxi test mode
 *
 * @unit deg
 * @min 0.0
 * @max 45.0
 * @decimal 1
 * @increment 1.0
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_XMAX, 35.0f);

/**
 * Maximum yaw setpoint slew rate in runway taxi test mode
 *
 * Limits the commanded heading change while taxiing so the nose wheel is not
 * fed a step change at waypoint transitions.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 1.0
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_YRMAX, 30.0f);

/**
 * Maximum runway taxi test duration
 *
 * If greater than zero, the open-loop taxi throttle is reduced to idle after
 * this many seconds. The nose-wheel controller remains enabled until the mode
 * is changed or the vehicle is disarmed.
 *
 * @unit s
 * @min 0.0
 * @decimal 1
 * @increment 1.0
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_TAXI_TIME, 0.0f);

/**
 * NPFG period while steering on runway
 *
 * @unit s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_NPFG_PERIOD, 5.0f);

/**
 * Enable use of yaw stick for nudging the wheel during runway ground roll
 *
 * This is useful when map, GNSS, or yaw errors on ground are misaligned with what the operator intends for takeoff course.
 * Particularly useful for skinny runways or if the wheel servo is a bit off trim.
 *
 * @boolean
 * @group Runway Takeoff
 */
PARAM_DEFINE_INT32(RWTO_NUDGE, 1);

/**
 * Takeoff rotation airspeed
 *
 * The calibrated airspeed threshold during the takeoff ground roll when the plane should start rotating (pitching up).
 * Must be less than the takeoff airspeed, will otherwise be capped at the takeoff airpeed (see FW_TKO_AIRSPD).
 *
 * If set <= 0.0, defaults to 0.9 * takeoff airspeed (see FW_TKO_AIRSPD)
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_ROT_AIRSPD, -1.0f);

/**
 * Takeoff rotation time
 *
 * This is the time desired to linearly ramp in takeoff pitch constraints during the takeoff rotation
 *
 * @unit s
 * @min 0.1
 * @decimal 1
 * @increment 0.1
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_ROT_TIME, 1.0f);

/**
 * Maximum pitch during runway takeoff
 *
 * A negative value uses FW_P_LIM_MAX. A non-negative value is active near the
 * runway and is continuously released with height by FW_P_TKO_HGT.
 *
 * @unit deg
 * @min -1.0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_PMAX, -1.0f);

/**
 * Height to keep nose-wheel yaw control after rotation starts
 *
 * A positive value keeps wheel yaw control active through CLIMBOUT until the
 * vehicle rises above this height. Zero preserves the legacy behavior.
 *
 * @unit m
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Runway Takeoff
 */
PARAM_DEFINE_FLOAT(RWTO_WHEEL_HGT, 0.0f);
