# 新增+2026-09-16：LESO 状态、输入及运行参数独立日志话题。
uint64 timestamp
# 新增+2026-09-16：本条 LESO 数据对应的测力样本处理时间戳。
uint64 timestamp_sample
# 新增+2026-09-16：四路用于 LESO 更新的实际升力测量值，单位 N。
float32[4] thrust_measure
# 新增+2026-09-16：四路 LESO 内部升力状态估计 y_hat，单位 N。
float32[4] y_hat
# 新增+2026-09-16：四路 LESO 总升力导数残差估计 d_hat，单位 N/s。
float32[4] d_hat
# 新增+2026-09-16：四路测量创新量 thrust_measure-y_hat，单位 N。
float32[4] innovation
# 新增+2026-09-16：四路名义升力导数 f_x+g_x*u_applied_prev，单位 N/s。
float32[4] nominal_t_dot
# 新增+2026-09-16：四路内部升力状态导数，单位 N/s。
float32[4] y_hat_dot
# 新增+2026-09-16：四路扰动状态导数，单位 N/s²。
float32[4] d_hat_dot
# 新增+2026-09-16：四路 mixer 斜率限制后输出反算的上一周期模型输入，范围 [0,1]。
float32[4] u_applied_prev
# 新增+2026-09-16：四路实际生效的 LESO 带宽 omega_o，单位 rad/s。
float32[4] omega_o_active
# 新增+2026-09-16：四路带宽覆盖参数原值；非正值代表回退至全局默认值。
float32[4] omega_o_override
# 新增+2026-09-16：LESO 全局默认带宽，单位 rad/s。
float32 omega_o_default
# 新增+2026-09-16：d_hat 内部状态的绝对限幅，单位 N/s。
float32 d_state_limit
# 新增+2026-09-16：本次 LESO 更新采用的相邻测力样本时间间隔，单位 s。
float32 dt
# 新增+2026-09-16：LESO 总开关状态，1 表示启用，0 表示关闭。
uint8 enabled
# 新增+2026-09-16：四路是否已接收到有效的 mixer 最终输出。
uint8[4] u_applied_valid
# 新增+2026-09-16：四路本轮是否随新测力样本完成 LESO 更新。
uint8[4] updated
# 新增+2026-09-16：四路 LESO 是否已完成首次有效样本初始化。
uint8[4] initialized
# 本周期实际用于控制律的 LESO 增益。
float32 kd_active
# 本周期 d_hat 进入控制律前的限幅值；非正值表示补偿关闭。
float32 d_comp_limit
# 本周期 LESO 主动补偿总开关是否真正满足使能条件。
uint8 compensation_enabled
# 四路本周期实际送入 IOLC 的 d_comp。
float32[4] d_comp_used
# 四路 LESO 对模型输入的理论改变量：
# delta_u = -kd_active * d_comp_used / g_x。
float32[4] delta_u_leso_unsat
# 新增+2026-09-16：将该消息注册为 ULog 话题。
# TOPICS thrust_leso_data
