# uint64 timestamp    # time since system start (microseconds)

# float32 thrust_error1
# float32 thrust_error2
# float32 thrust_error3
# float32 thrust_error4

# float32 thrust_control_out1
# float32 thrust_control_out2
# float32 thrust_control_out3
# float32 thrust_control_out4

# float32 thrust_desired1
# float32 thrust_desired2
# float32 thrust_desired3
# float32 thrust_desired4

# float32 thrust_desired_dot1
# float32 thrust_desired_dot2
# float32 thrust_desired_dot3
# float32 thrust_desired_dot4

# float32 thrust_control_out_pwm1
# float32 thrust_control_out_pwm2
# float32 thrust_control_out_pwm3
# float32 thrust_control_out_pwm4

# float32 thrust_feedforward_out1
# float32 thrust_feedforward_out2
# float32 thrust_feedforward_out3
# float32 thrust_feedforward_out4

# float32 thrust_feedback_out1
# float32 thrust_feedback_out2
# float32 thrust_feedback_out3
# float32 thrust_feedback_out4

# float32 thrust_kalman_filter_control_out_pwm1
# float32 thrust_kalman_filter_control_out_pwm2
# float32 thrust_kalman_filter_control_out_pwm3
# float32 thrust_kalman_filter_control_out_pwm4

# float32 motor_speed_dot1
# float32 motor_speed_dot2
# float32 motor_speed_dot3
# float32 motor_speed_dot4

# float32 motor_speed1
# float32 motor_speed2
# float32 motor_speed3
# float32 motor_speed4

# float32 f_x1
# float32 f_x2
# float32 f_x3
# float32 f_x4

# float32 g_x1
# float32 g_x2
# float32 g_x3
# float32 g_x4

# float32 deta_t_1
# float32 deta_t_2
# float32 deta_t_3
# float32 deta_t_4

# float32 thrust_start

# float32 kp
# float32 ki
# float32 kd

# float32 err_integral1
# float32 err_integral2
# float32 err_integral3
# float32 err_integral4

# float32 virtual_control1
# float32 virtual_control2
# float32 virtual_control3
# float32 virtual_control4

# int32 sensor_model

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

# float32 kp
# float32 ki
# float32 kd

float32[4] err_integral

float32[4] virtual_control

int32 sensor_model

# Estimated derivative of measured thrust for motors 1-4
float32[4] thrust_measure_dot

float32[4] fb_p_part
float32[4] fb_i_part
float32[4] fb_nolin_part

# TOPICS thrust_control_data
