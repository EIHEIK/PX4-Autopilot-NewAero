uint32 MESSAGE_VERSION = 1

uint64 timestamp		# time since system start (microseconds)

float32 yaw_sp_move_rate	# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d			# Desired quaternion for quaternion control

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body		# Normalized thrust command in body FRD frame [-1,1]

bool reset_integral	# Reset roll/pitch/yaw integrals (navigation logic change)

bool fw_control_yaw_wheel	# control heading with steering wheel (used for auto takeoff on runway)

# Optional fixed-wing phase information. The normal value and zero blend keep
# legacy publishers/controllers unchanged. Fixed-wing position control uses the
# blend to move continuously between near-ground and cruise pitch-rate limits.
uint8 FW_PITCH_MODE_NORMAL = 0
uint8 FW_PITCH_MODE_TAKEOFF = 1
uint8 FW_PITCH_MODE_LANDING = 2
uint8 fw_pitch_mode
float32 fw_pitch_limit_blend	# 0: cruise authority, 1: phase-specific near-ground authority

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
