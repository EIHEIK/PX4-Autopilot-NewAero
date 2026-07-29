uint64 timestamp                         # time since system start [us]
uint64 timestamp_sample                  # Gazebo diagnostic sample time [us]

float32 target_throttle                  # ESC target [0,1]
float32 filtered_throttle                # engine first-order state [0,1]
float32 altitude_m                       # altitude used by the propulsion table [m]
float32 airspeed_m_s                     # airspeed used by the propulsion table [m/s]
float32 rpm                              # interpolated propeller speed [rpm]
float32 thrust_n                         # interpolated thrust [N]
float32 torque_nm                        # interpolated shaft torque [N m]
float32 fuel_rate                        # diagnostic fuel-table output
uint32 flags                             # HonghuPropulsionV8 diagnostic flags
uint32 sequence                          # source diagnostic sequence number
