uint64 timestamp    # time since system start (microseconds)

float32 thrust_desired1
float32 thrust_desired2
float32 thrust_desired3
float32 thrust_desired4

float32 torque_pitch_desired

float32 open_loop_control_output1
float32 open_loop_control_output2
float32 open_loop_control_output3
float32 open_loop_control_output4

float32 full_process_output1
float32 full_process_output2
float32 full_process_output3
float32 full_process_output4

# 新增+2026-09-16：4通道 经 mixer 斜率限制后的最终归一化 PWM 输出，范围 [-1,1]。
float32[4] final_output_post_slew
