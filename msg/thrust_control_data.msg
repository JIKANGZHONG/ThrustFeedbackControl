uint64 timestamp    # time since system start (microseconds)

float32[4] thrust_error
float32[4] thrust_control_out
float32[4] thrust_desired
float32[4] thrust_desired_dot
float32[4] thrust_control_out_pwm
float32[4] thrust_feedforward_out
float32[4] thrust_feedback_out
float32[4] thrust_kalman_filter_control_out_pwm
float32[4] motor_speed_dot
float32[4] motor_speed
float32[4] f_x
float32[4] g_x
float32[4] deta_t
float32 thrust_start

float32[4] err_integral
float32[4] virtual_control
int32 sensor_model
# Estimated derivative of measured thrust for motors 1-4
float32[4] thrust_measure_dot

float32[4] fb_p_part
float32[4] fb_i_part
float32[4] fb_nolin_part
# 新增+2026-09-18：实际总输入域中的期望推力导数贡献 u_D。
float32[4] fb_tdot_part
# 新增+2026-09-18：实际总输入域中的 LESO 补偿贡献 u_LESO。
float32[4] fb_leso_part
# 新增+2026-09-18：内部 [0, u_max] 限幅前的总控制输入 u_raw。
float32[4] u_raw
# 新增+2026-09-18：本周期 g_x 重标定前的积分状态。
float32[4] err_integral_before_rescale
# 新增+2026-09-18：g_x 重标定后、叠加本周期 e*dt 前的积分状态。
float32[4] err_integral_after_rescale
# 新增+2026-09-18：用于本周期重标定的上一有效周期 g_x。
float32[4] g_x_prev
# 新增+2026-09-18：本周期实际采用的积分重标定比例。
float32[4] g_x_rescale_ratio

# TOPICS thrust_control_data
