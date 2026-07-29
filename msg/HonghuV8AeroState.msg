uint64 timestamp                         # time since system start [us]
uint64 timestamp_sample                  # Gazebo diagnostic sample time [us]

float32 airspeed_m_s                     # relative-air velocity magnitude [m/s]
float32 alpha_deg                        # PDF/PX4 FRD angle of attack [deg]
float32 beta_deg                         # PDF/PX4 FRD sideslip angle [deg]
float32 rho_kg_m3                        # density used by the V8 plugin [kg/m^3]
float32 alpha_dot_rad_s                  # filtered angle-of-attack rate [rad/s]
float32 beta_dot_rad_s                   # filtered sideslip rate [rad/s]
float32[3] body_rates_frd_rad_s          # p, q, r in PX4 FRD [rad/s]
float32[6] coefficients                  # CL, CD, CY, Cl, Cm, Cn
float32[8] joint_angles_deg              # actual Gazebo theta_joint for servo_0..7 [deg]
float32[4] delta_doc_deg                 # delta_a, delta_e, delta_r, delta_c [deg]
uint32 flags                             # HonghuAeroV8 diagnostic flags
uint32 sequence                          # source diagnostic sequence number
