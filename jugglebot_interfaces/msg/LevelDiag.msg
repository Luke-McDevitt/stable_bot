# Per-tick diagnostic snapshot of stewart_control_node._level_run.
# Published at ~50 Hz while the level loop is active. See
# stewart_bringup/docs/level_pi_tuning_plan.md for field rationale.

# Timestamps (monotonic seconds; not ROS time — for measuring loop jitter)
float64 t_imu          # When the IMU sample driving this tick was received
float64 t_tick         # When this controller iteration started

# Raw IMU reading this tick (degrees)
float32 roll
float32 pitch

# Target the controller is trying to hold
# = level_ref + current_rpy + step_offset
float32 target_roll
float32 target_pitch

# Errors (degrees)
float32 err_r          # raw err = roll - target_roll
float32 err_p
float32 err_r_filt     # alpha-LPF of err_r
float32 err_p_filt

# Integrator state (degrees, after back-calc anti-windup + clamp)
float32 integ_r
float32 integ_p

# Unclipped PI output (= -err_filt * KP + integ). Diverges from corr
# when the rate-limit or MAX_CORR clamp engages — that's the signal
# the controller is fighting saturation.
float32 pi_out_r
float32 pi_out_p

# Final command after rate-limit + MAX_CORR clamp (degrees)
float32 corr_r
float32 corr_p

# Bitfield. Bit 0: rate_limit_r, 1: rate_limit_p,
#           2: max_corr_r,    3: max_corr_p,
#           4: deadband_r,    5: deadband_p,
#           6: integ_clamp_r, 7: integ_clamp_p
uint16 clip_flags

# Per-leg state, 6 entries each
float64[6] motor_targets   # commanded leg position (turns) sent to feeder
float64[6] leg_enc         # actual leg position from encoders (turns); NaN if stale
float64[6] leg_iq          # measured Iq (Amps); NaN if stale

# Loop jitter — period seconds of this tick. Should be ~0.020.
float32 dt_actual

# Commanded platform pose this tick (xyz mm, rpy degrees) — captures Z so
# bags from different operating points are self-describing.
float32[6] target_xyzrpy
