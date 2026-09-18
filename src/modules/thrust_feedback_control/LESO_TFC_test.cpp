/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include "thrust_feedback_control.hpp"

// Low-pass filters used by the original controller implementation
math::LowPassFilter2p<float>	_position_pid_pout_lowpass_filter{100.f, 30.f};
math::LowPassFilter2p<float>	_position_pid_dout_lowpass_filter{100.f, 10.f};

px4::AppState ThrustFeedbackControl::appState;

math::LowPassFilter2p<float>	_thrust_desired_1_lowpass_filter{100.f, 5.f};
math::LowPassFilter2p<float>	_thrust_desired_2_lowpass_filter{100.f, 5.f};
math::LowPassFilter2p<float>	_thrust_desired_3_lowpass_filter{100.f, 5.f};
math::LowPassFilter2p<float>	_thrust_desired_4_lowpass_filter{100.f, 5.f};

math::LowPassFilter2p<float>    _output_1_lowpass_filter{100.f, 15.f};
math::LowPassFilter2p<float>    _output_2_lowpass_filter{100.f, 15.f};
math::LowPassFilter2p<float>    _output_3_lowpass_filter{100.f, 15.f};
math::LowPassFilter2p<float>    _output_4_lowpass_filter{100.f, 15.f};

/* IOLC state and parameters for motor 1 */
typedef struct
{
    uint64_t last_timestamp;

	// Nonlinear motor/thrust model coefficients
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff, u_fb;
	double dt;
    float thrust_desired_dot, thrust_desired;
    double motorSpeed_ff;
    float u_fb_coeff;
    double f_x, g_x;
    double u_max;
    double motor_speed_max;

}IOLC;

/* IOLC state and parameters for motor 2 */
typedef struct
{
    uint64_t last_timestamp;

	// Nonlinear motor/thrust model coefficients
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff, u_fb;
	double dt;
    float thrust_desired_dot, thrust_desired;
    double motorSpeed_ff;
    float u_fb_coeff;
    double f_x, g_x;
    double u_max;
    double motor_speed_max;

}IOLC2;

/* IOLC state and parameters for motor 3 */
typedef struct
{
    uint64_t last_timestamp;

	// Nonlinear motor/thrust model coefficients
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff, u_fb;
	double dt;
    float thrust_desired_dot, thrust_desired;
    double motorSpeed_ff;
    float u_fb_coeff;
    double f_x, g_x;
    double u_max;
    double motor_speed_max;
}IOLC3;

/* IOLC state and parameters for motor 4 */
typedef struct
{
    uint64_t last_timestamp;

	// Nonlinear motor/thrust model coefficients
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff, u_fb;
	double dt;
    float thrust_desired_dot, thrust_desired;
    double motorSpeed_ff;
    float u_fb_coeff;
    double f_x, g_x;
    double u_max;
    double motor_speed_max;

}IOLC4;

// 新增+2026-09-16：四路 LESO 共用的单旋翼观测器状态；仅用于观测与日志，当前不参与控制律。
typedef struct
{
    // 新增+2026-09-16：LESO 内部升力估计 y_hat，单位与 _thrust_measure 一致。
    float y_hat;
    // 新增+2026-09-16：总升力导数残差估计 d_hat，单位为升力单位/s。
    float d_hat;
    // 新增+2026-09-16：观测创新量 T_meas-y_hat，供日志分析与调参使用。
    float innovation;
    // 新增+2026-09-16：名义升力导数 f_x+g_x*u_applied_prev，单位为升力单位/s。
    float nominal_t_dot;
    // 新增+2026-09-16：内部升力估计导数，供日志分析与数值检查使用。
    float y_hat_dot;
    // 新增+2026-09-16：扰动估计导数，供日志分析与数值检查使用。
    float d_hat_dot;
    // 新增+2026-09-16：mixer 斜率限制后上一周期输出反算得到的模型输入 u_applied_prev。
    float u_applied_prev;
    // 新增+2026-09-16：首个有效观测样本已完成状态初始化的标志。
    bool initialized;
} ThrustLESO;

// 新增+2026-09-16：按新测力样本执行一次二阶 LESO；返回 true 表示本周期完成有效更新。
static bool LESO_Update(ThrustLESO *leso, float thrust_meas, float a_hat, float b_hat,
                        float u_applied_prev, float dt, float omega_o, float d_state_limit)
{
    // 新增+2026-09-16：任何非法输入、未启用带宽或未配置状态限幅时均保持旧状态，防止错误数据进入观测器。
    if (leso == nullptr || !PX4_ISFINITE(thrust_meas) || !PX4_ISFINITE(a_hat)
        || !PX4_ISFINITE(b_hat) || !PX4_ISFINITE(u_applied_prev) || !PX4_ISFINITE(dt)
        || !PX4_ISFINITE(omega_o) || !PX4_ISFINITE(d_state_limit)
        || dt <= 0.0f || omega_o <= 0.0f || d_state_limit <= 0.0f) {
        return false;
    }

    // 新增+2026-09-16：首次有效样本以实测升力初始化 y_hat，并令未知总残差初值为零。
    if (!leso->initialized) {
        leso->y_hat = thrust_meas;
        leso->d_hat = 0.0f;
        leso->initialized = true;
    }

    // 新增+2026-09-16：采用 beta1=2*omega_o、beta2=omega_o^2 的带宽参数化 LESO。
    const float beta1 = 2.0f * omega_o;
    // 新增+2026-09-16：二阶 LESO 的扰动观测增益 beta2。
    const float beta2 = omega_o * omega_o;
    // 新增+2026-09-16：当前测力相对内部升力状态的差。
    const float innovation = thrust_meas - leso->y_hat;
    // 新增+2026-09-16：使用 mixer 实际输出坐标下的名义升力导数预测。
    const float nominal_t_dot = a_hat + b_hat * u_applied_prev;
    // 新增+2026-09-16：LESO 升力状态导数。
    const float y_hat_dot = nominal_t_dot + leso->d_hat + beta1 * innovation;
    // 新增+2026-09-16：LESO 总残差状态导数。
    const float d_hat_dot = beta2 * innovation;
    // 新增+2026-09-16：前向 Euler 离散后的候选升力状态。
    const float y_hat_next = leso->y_hat + dt * y_hat_dot;
    // 新增+2026-09-16：前向 Euler 离散后的候选总残差状态，并执行用户配置的安全限幅。
    const float d_hat_next = math::constrain(leso->d_hat + dt * d_hat_dot,
                             -d_state_limit, d_state_limit);

    // 新增+2026-09-16：拒绝计算中产生的非有限状态，保证 observer-only 阶段也不会污染日志或后续控制接口。
    if (!PX4_ISFINITE(y_hat_next) || !PX4_ISFINITE(d_hat_next)
        || !PX4_ISFINITE(nominal_t_dot) || !PX4_ISFINITE(y_hat_dot) || !PX4_ISFINITE(d_hat_dot)) {
        return false;
    }

    // 新增+2026-09-16：提交本周期 LESO 状态与调试量。
    leso->innovation = innovation;
    leso->nominal_t_dot = nominal_t_dot;
    leso->y_hat_dot = y_hat_dot;
    leso->d_hat_dot = d_hat_dot;
    leso->u_applied_prev = u_applied_prev;
    leso->y_hat = y_hat_next;
    leso->d_hat = d_hat_next;
    return true;
}

// Apply the LESO correction to the existing feedforward + weighted-feedback
// IOLC structure. With a valid feedback weight this is algebraically
// equivalent to u = u_ff + k_fb * (v - f_x - k_d*d_comp/k_fb) / g_x,
// so the total motor-model input changes by exactly -k_d*d_comp/g_x.
// Invalid inputs, a zero/small feedback weight, or k_d == 0 retain the
// original controller behaviour.
static void IOLC_ComputeOutput(double v, double f_x, double g_x, double u_ff,
                               float u_fb_coeff, float leso_kd, float leso_d_comp,
                               double u_max, double &u_fb, double &u)
{
    constexpr double kMinAbsGain = 1e-9;
    constexpr float kMinAbsFeedbackWeight = 1e-6f;

    u_fb = 0.0;

    const bool nominal_valid = PX4_ISFINITE(v) && PX4_ISFINITE(f_x)
                               && PX4_ISFINITE(g_x) && fabs(g_x) > kMinAbsGain;

    if (nominal_valid) {
        u_fb = (v - f_x) / g_x;

        const bool leso_compensation_valid = PX4_ISFINITE(leso_kd)
                                             && PX4_ISFINITE(leso_d_comp)
                                             && leso_kd > 0.0f
                                             && fabsf(u_fb_coeff) > kMinAbsFeedbackWeight;

        if (leso_compensation_valid) {
            u_fb -= (static_cast<double>(leso_kd) * static_cast<double>(leso_d_comp))
                    / (static_cast<double>(u_fb_coeff) * g_x);
        }
    }

    const double safe_u_ff = PX4_ISFINITE(u_ff) ? u_ff : 0.0;
    u = safe_u_ff + static_cast<double>(u_fb_coeff) * u_fb;

    if (!PX4_ISFINITE(u)) {
        u = 0.0;
    }

    u = (u > u_max) ? u_max : ((u < 0.0) ? 0.0 : u);
}

/* Calculate IOLC command and update internal motor-speed model for motor 1 */
void IOLC_Calculate(IOLC *iolc, float leso_kd, float leso_d_comp)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // Virtual control law: desired thrust derivative + PI tracking correction
    //iolc->err_integral += iolc->err * static_cast<float>(dt);
    if (iolc->thrust_desired > 1e-3f) {
        iolc->err_integral += iolc->err * static_cast<float>(dt);
    }
    else {
        iolc->err_integral = 0.0f;
    }
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);

    iolc->v = iolc->thrust_desired_dot + iolc->kp * iolc->err + iolc->ki * iolc->err_integral;

    // Input-output linearization: calculate nonlinear drift term f(x) and input gain g(x)
	iolc->f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	iolc->g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    IOLC_ComputeOutput(iolc->v, iolc->f_x, iolc->g_x, iolc->u_ff,
                       iolc->u_fb_coeff, leso_kd, leso_d_comp, iolc->u_max,
                       iolc->u_fb, iolc->u);


	// Euler integration of the internal motor-speed model
	iolc->motorSpeed_dot = (iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + iolc->motorSpeed_dot*dt;
	//iolc->motorSpeed = (iolc->motorSpeed > 4000) ? 4000 : ((iolc->motorSpeed < 100) ? 100 : iolc->motorSpeed);
    // iolc->motorSpeed = (iolc->motorSpeed > 5000) ? 5000 : ((iolc->motorSpeed < 100) ? 100 : iolc->motorSpeed);
    iolc->motorSpeed = (iolc->motorSpeed > iolc->motor_speed_max) ? iolc->motor_speed_max : ((iolc->motorSpeed < 100.0) ? 100.0 : iolc->motorSpeed);
}

/* Calculate IOLC command and update internal motor-speed model for motor 2 */
void IOLC_Calculate2(IOLC2 *iolc, float leso_kd, float leso_d_comp)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    //iolc->err_integral += iolc->err * static_cast<float>(dt);
    if (iolc->thrust_desired > 1e-3f) {
        iolc->err_integral += iolc->err * static_cast<float>(dt);
    }
    else {
        iolc->err_integral = 0.0f;
    }
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);

    iolc->v = iolc->thrust_desired_dot + iolc->kp * iolc->err + iolc->ki * iolc->err_integral;

	iolc->f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	iolc->g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    IOLC_ComputeOutput(iolc->v, iolc->f_x, iolc->g_x, iolc->u_ff,
                       iolc->u_fb_coeff, leso_kd, leso_d_comp, iolc->u_max,
                       iolc->u_fb, iolc->u);

	iolc->motorSpeed_dot = (iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + iolc->motorSpeed_dot*dt;
	//iolc->motorSpeed = (iolc->motorSpeed > 4000) ? 4000 : ((iolc->motorSpeed < 100) ? 100 : iolc->motorSpeed);
    iolc->motorSpeed = (iolc->motorSpeed > iolc->motor_speed_max) ? iolc->motor_speed_max : ((iolc->motorSpeed < 100.0) ? 100.0 : iolc->motorSpeed);
}

/* Calculate IOLC command and update internal motor-speed model for motor 3 */
void IOLC_Calculate3(IOLC3 *iolc, float leso_kd, float leso_d_comp)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

 //iolc->err_integral += iolc->err * static_cast<float>(dt);
    if (iolc->thrust_desired > 1e-3f) {
        iolc->err_integral += iolc->err * static_cast<float>(dt);
    }
    else {
        iolc->err_integral = 0.0f;
    }
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);

    iolc->v = iolc->thrust_desired_dot + iolc->kp * iolc->err + iolc->ki * iolc->err_integral;

	iolc->f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	iolc->g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    IOLC_ComputeOutput(iolc->v, iolc->f_x, iolc->g_x, iolc->u_ff,
                       iolc->u_fb_coeff, leso_kd, leso_d_comp, iolc->u_max,
                       iolc->u_fb, iolc->u);

	iolc->motorSpeed_dot = (iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + iolc->motorSpeed_dot*dt;
	//iolc->motorSpeed = (iolc->motorSpeed > 4000) ? 4000 : ((iolc->motorSpeed < 100) ? 100 : iolc->motorSpeed);
    iolc->motorSpeed = (iolc->motorSpeed > iolc->motor_speed_max) ? iolc->motor_speed_max : ((iolc->motorSpeed < 100.0) ? 100.0 : iolc->motorSpeed);
}

/* Calculate IOLC command and update internal motor-speed model for motor 4 */
void IOLC_Calculate4(IOLC4 *iolc, float leso_kd, float leso_d_comp)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    //iolc->err_integral += iolc->err * static_cast<float>(dt);
    if (iolc->thrust_desired > 1e-3f) {
        iolc->err_integral += iolc->err * static_cast<float>(dt);
    }
    else {
        iolc->err_integral = 0.0f;
    }
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);

    iolc->v = iolc->thrust_desired_dot + iolc->kp * iolc->err + iolc->ki * iolc->err_integral;

	iolc->f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	iolc->g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    IOLC_ComputeOutput(iolc->v, iolc->f_x, iolc->g_x, iolc->u_ff,
                       iolc->u_fb_coeff, leso_kd, leso_d_comp, iolc->u_max,
                       iolc->u_fb, iolc->u);

	iolc->motorSpeed_dot = (iolc->a3*iolc->motorSpeed*iolc->motorSpeed*iolc->motorSpeed + iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + iolc->motorSpeed_dot*dt;
	//iolc->motorSpeed = (iolc->motorSpeed > 4000) ? 4000 : ((iolc->motorSpeed < 100) ? 100 : iolc->motorSpeed);
    iolc->motorSpeed = (iolc->motorSpeed > iolc->motor_speed_max) ? iolc->motor_speed_max : ((iolc->motorSpeed < 100.0) ? 100.0 : iolc->motorSpeed);

}

/* Update module parameters when parameter_update is published */
void ThrustFeedbackControl::parameters_update()
{
    if (_parameter_update_sub.updated()) {

        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
    }
}

int ThrustFeedbackControl::main()
{
    appState.setRunning(true);

    // Subscribe to four-channel force-sensor raw data
    int thrustdata_sub_fd = orb_subscribe(ORB_ID(barometric_force_sensor));

    // Limit sensor subscription update interval to 10 ms (100 Hz)
    orb_set_interval(thrustdata_sub_fd, 10);

    // Subscribe to desired thrust/open-loop control data from the flight controller
    int thrustdesireddata_sub_fd = orb_subscribe(ORB_ID(thrust_desired_data));
    orb_set_interval(thrustdesireddata_sub_fd, 10);

    // PX4 actuator armed state.
    // This is used only to guarantee that automatic force-sensor
    // zero calibration is performed while the vehicle is disarmed.
    // int actuator_armed_sub_fd = orb_subscribe(ORB_ID(actuator_armed));

    // Poll all three topics.
    // fds[0] : force sensor
    // fds[1] : desired thrust
    // fds[2] : armed state
    px4_pollfd_struct_t fds[] =
    {
        { .fd = thrustdata_sub_fd,   .events = POLLIN },
        { .fd = thrustdesireddata_sub_fd,   .events = POLLIN },
        // { .fd = actuator_armed_sub_fd, .events = POLLIN },
    };

    int error_counter = 0;
    struct barometric_force_sensor_s sensordata{};
    struct thrust_desired_data_s thrustdesireddata{};
    // struct actuator_armed_s actuator_armed{};
    // bool armed_state_known = false;
    // Independent IOLC states for four motors
    static IOLC _iolc = {};
    static IOLC2 _iolc2 = {};
    static IOLC3 _iolc3 = {};
    static IOLC4 _iolc4 = {};
    static ThrustKalmanFilter thrust_kalman_filter_measure{1.0f};
    // Estimated derivative of the thrust measurement used by the controller
    float thrust_measure_dot[4] = {};
    // 新增+2026-09-16：四个旋翼各自独立的 LESO 状态；observer-only 阶段不改变现有 TFC 输出。
    ThrustLESO leso[4] = {};
    // 新增+2026-09-16：mixer 斜率限制后上一周期实际 PWM 反算得到的四路模型输入。
    float leso_u_applied_prev[4] = {};
    // 新增+2026-09-16：对应通道是否已经接收到有效的 mixer 最终输出，避免启动阶段使用未知输入更新 LESO。
    bool leso_u_applied_valid[4] = {};
    // 新增+2026-09-16：记录每个旋翼本轮是否完成 LESO 更新，供 ULog 对齐与故障分析使用。
    bool leso_updated[4] = {};
    // True after at least one valid actuator_armed message has been received.
    // Calibration is not allowed before the arming state is known.


    // 新增+2026-09-16：LESO 独立日志消息缓存；static 避免占用 main 函数栈空间。
    static thrust_leso_data_s thrustlesodata = {};
    // 新增+2026-09-16：LESO 独立日志发布器；仅在本控制器主循环内使用。
    static uORB::Publication<thrust_leso_data_s> thrustlesodata_pub{ORB_ID(thrust_leso_data)};


    // Initialize the nonlinear IOLC model parameters for all four motors
    _iolc.a1 = iolc_a1;
    _iolc.a2 = iolc_a2;
    _iolc.a3 = iolc_a3;
    _iolc.b0 = iolc_b0;
    _iolc.c1 = iolc_c1;
    _iolc.c2 = iolc_c2;
    _iolc.c3 = iolc_c3;
    _iolc.d1 = iolc_d1;
    _iolc.d2 = iolc_d2;
    _iolc.motorSpeed = motorSpeed_FF;

    _iolc2.a1 = iolc_a1;
    _iolc2.a2 = iolc_a2;
    _iolc2.a3 = iolc_a3;
    _iolc2.b0 = iolc_b0;
    _iolc2.c1 = iolc_c1;
    _iolc2.c2 = iolc_c2;
    _iolc2.c3 = iolc_c3;
    _iolc2.d1 = iolc_d1;
    _iolc2.d2 = iolc_d2;
    _iolc2.motorSpeed = motorSpeed_FF;

    _iolc3.a1 = iolc_a1;
    _iolc3.a2 = iolc_a2;
    _iolc3.a3 = iolc_a3;
    _iolc3.b0 = iolc_b0;
    _iolc3.c1 = iolc_c1;
    _iolc3.c2 = iolc_c2;
    _iolc3.c3 = iolc_c3;
    _iolc3.d1 = iolc_d1;
    _iolc3.d2 = iolc_d2;
    _iolc3.motorSpeed = motorSpeed_FF;

    _iolc4.a1 = iolc_a1;
    _iolc4.a2 = iolc_a2;
    _iolc4.a3 = iolc_a3;
    _iolc4.b0 = iolc_b0;
    _iolc4.c1 = iolc_c1;
    _iolc4.c2 = iolc_c2;
    _iolc4.c3 = iolc_c3;
    _iolc4.d1 = iolc_d1;
    _iolc4.d2 = iolc_d2;
    _iolc4.motorSpeed = motorSpeed_FF;

    while(appState.isRunning())
    {
        parameters_update();

        // These variables ensure the measured-thrust derivative filter is updated
        // only when a new force-sensor sample is received.
        bool thrust_measure_updated = false;
        float thrust_measure_dt = 0.0f;

        int poll_ret = px4_poll(fds, 2, 1000);

        if (poll_ret == 0)
        {
            PX4_ERR("Got no data within a second");
        }
        else if (poll_ret < 0)
        {
            if (error_counter < 10 || error_counter % 50 ==0)
            {
                PX4_ERR("ERROR RETURN VALUE FROM POLL(): %d", poll_ret);
            }
            error_counter++;
        }
        else
        {
            // New force-sensor sample available
            if (fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(barometric_force_sensor), thrustdata_sub_fd, &sensordata);

                thrustdata.timestamp = hrt_absolute_time();
                static uint64_t last_timestamp = thrustdata.timestamp;

                thrustdata.thrust_raw_data[0] = sensordata.data1 / 1000.0f;
                thrustdata.thrust_raw_data[1] = sensordata.data2 / 1000.0f;
                thrustdata.thrust_raw_data[2] = sensordata.data3 / 1000.0f;
                thrustdata.thrust_raw_data[3] = sensordata.data4 / 1000.0f;

                // Model 1: original bias1 + bias2 compensation
                // thrustdata.thrust_raw_comp[0] = (sensordata.data1 + _param_sensor1_bias1.get() + _param_sensor1_bias2.get()) / 1000.0f;
                // thrustdata.thrust_raw_comp[1] = (sensordata.data2 + _param_sensor2_bias1.get() + _param_sensor2_bias2.get()) / 1000.0f;
                // thrustdata.thrust_raw_comp[2] = (sensordata.data3 + _param_sensor3_bias1.get() + _param_sensor3_bias2.get()) / 1000.0f;
                // thrustdata.thrust_raw_comp[3] = (sensordata.data4 + _param_sensor4_bias1.get() + _param_sensor4_bias2.get()) / 1000.0f;
                thrustdata.thrust_raw_comp[0] = (sensordata.data1 + _param_sensor1_bias1.get()) / 1000.0f;
                thrustdata.thrust_raw_comp[1] = (sensordata.data2 + _param_sensor2_bias1.get()) / 1000.0f;
                thrustdata.thrust_raw_comp[2] = (sensordata.data3 + _param_sensor3_bias1.get()) / 1000.0f;
                thrustdata.thrust_raw_comp[3] = (sensordata.data4 + _param_sensor4_bias1.get()) / 1000.0f;
                // Model 1: original bias1 + bias2 compensation
                thrustdata.thrust_raw_comp[0] = (thrustdata.thrust_raw_comp[0] > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_comp[0] < 0.0f) ? 0.0f : thrustdata.thrust_raw_comp[0]);
                thrustdata.thrust_raw_comp[1] = (thrustdata.thrust_raw_comp[1] > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_comp[1] < 0.0f) ? 0.0f : thrustdata.thrust_raw_comp[1]);
                thrustdata.thrust_raw_comp[2] = (thrustdata.thrust_raw_comp[2] > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_comp[2] < 0.0f) ? 0.0f : thrustdata.thrust_raw_comp[2]);
                thrustdata.thrust_raw_comp[3] = (thrustdata.thrust_raw_comp[3] > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_comp[3] < 0.0f) ? 0.0f : thrustdata.thrust_raw_comp[3]);

                // Sensor sampling interval used by the thrust filters
                float dt = (float)(thrustdata.timestamp - last_timestamp) / 1000000.0f;
                thrust_measure_dt = dt;
                thrust_measure_updated = true;

                // Second-order Kalman filtering of model-1 thrust measurements
                thrustdata.thrust_kalman_filter_data_1 = thrust_kalman_filter.thrust_kalman_filter_BFS1(dt, thrustdata.thrust_raw_comp[0]);
                thrustdata.thrust_kalman_filter_data_2 = thrust_kalman_filter.thrust_kalman_filter_BFS2(dt, thrustdata.thrust_raw_comp[1]);
                thrustdata.thrust_kalman_filter_data_3 = thrust_kalman_filter.thrust_kalman_filter_BFS3(dt, thrustdata.thrust_raw_comp[2]);
                thrustdata.thrust_kalman_filter_data_4 = thrust_kalman_filter.thrust_kalman_filter_BFS4(dt, thrustdata.thrust_raw_comp[3]);
                // Second-order Kalman filtering of model-1 thrust measurements
                thrustdata.thrust_kalman_filter_data_1 = (thrustdata.thrust_kalman_filter_data_1 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_1 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_1);
                thrustdata.thrust_kalman_filter_data_2 = (thrustdata.thrust_kalman_filter_data_2 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_2 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_2);
                thrustdata.thrust_kalman_filter_data_3 = (thrustdata.thrust_kalman_filter_data_3 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_3 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_3);
                thrustdata.thrust_kalman_filter_data_4 = (thrustdata.thrust_kalman_filter_data_4 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_4 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_4);

                last_timestamp = thrustdata.timestamp;
                _thrustdata_pub.publish(thrustdata);
            }

            // New desired-thrust sample available
            if (fds[1].revents & POLLIN)
            {
                orb_copy(ORB_ID(thrust_desired_data), thrustdesireddata_sub_fd, &thrustdesireddata);

                // 新增+2026-09-16：读取 mixer 斜率限制后的最终 PWM，并反算为 IOLC 模型使用的归一化输入 u∈[0,1]。
                const float mixer_final_output[4] = {
                    thrustdesireddata.final_output_post_slew[0],
                    thrustdesireddata.final_output_post_slew[1],
                    thrustdesireddata.final_output_post_slew[2],
                    thrustdesireddata.final_output_post_slew[3]
                };

                // 新增+2026-09-16：仅在四路 mixer 最终输出均为有限值时更新缓存，避免不完整 uORB 消息污染 LESO。
                if (PX4_ISFINITE(mixer_final_output[0]) && PX4_ISFINITE(mixer_final_output[1])
                    && PX4_ISFINITE(mixer_final_output[2]) && PX4_ISFINITE(mixer_final_output[3])) {
                    // 新增+2026-09-16：通道 1 的 [-1,1] 最终 PWM 映射回 LESO 所需的 [0,1] 电机模型输入。
                    leso_u_applied_prev[0] = math::constrain(0.5f * (mixer_final_output[0] + 1.0f), 0.0f, 1.0f);
                    // 新增+2026-09-16：通道 2 的 [-1,1] 最终 PWM 映射回 LESO 所需的 [0,1] 电机模型输入。
                    leso_u_applied_prev[1] = math::constrain(0.5f * (mixer_final_output[1] + 1.0f), 0.0f, 1.0f);
                    // 新增+2026-09-16：通道 3 的 [-1,1] 最终 PWM 映射回 LESO 所需的 [0,1] 电机模型输入。
                    leso_u_applied_prev[2] = math::constrain(0.5f * (mixer_final_output[2] + 1.0f), 0.0f, 1.0f);
                    // 新增+2026-09-16：通道 4 的 [-1,1] 最终 PWM 映射回 LESO 所需的 [0,1] 电机模型输入。
                    leso_u_applied_prev[3] = math::constrain(0.5f * (mixer_final_output[3] + 1.0f), 0.0f, 1.0f);
                    // 新增+2026-09-16：标记四路实际输入均已就绪，可供新测力样本触发的 LESO 使用。
                    leso_u_applied_valid[0] = true;
                    leso_u_applied_valid[1] = true;
                    leso_u_applied_valid[2] = true;
                    leso_u_applied_valid[3] = true;
                }

                thrustdesireddata.timestamp = hrt_absolute_time();
                static uint64_t last_timestamp_dt = thrustdesireddata.timestamp;

                // Reject NaN/Inf and constrain desired normalized thrust to [0, 1]
                auto clean_desired = [](float raw, int idx) -> float {
                    if (!PX4_ISFINITE(raw)) {

                        return 0.0f;
                    }

                    return math::constrain(raw, 0.0f, 1.0f);
                };

                float des1 = clean_desired(thrustdesireddata.thrust_desired1, 1);
                float des2 = clean_desired(thrustdesireddata.thrust_desired2, 2);
                float des3 = clean_desired(thrustdesireddata.thrust_desired3, 3);
                float des4 = clean_desired(thrustdesireddata.thrust_desired4, 4);

                // Convert normalized controller demand to desired thrust
                _thrust_desired(0) = _param_tfc_pwm_to_thrust_factor1.get() * des1;
                _thrust_desired(1) = _param_tfc_pwm_to_thrust_factor2.get() * des2;
                _thrust_desired(2) = _param_tfc_pwm_to_thrust_factor3.get() * des3;
                _thrust_desired(3) = _param_tfc_pwm_to_thrust_factor4.get() * des4;

                _thrust_desired(0) = ( _thrust_desired(0) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(0) < 0.0f) ? 0.0f : _thrust_desired(0));
                _thrust_desired(1) = ( _thrust_desired(1) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(1) < 0.0f) ? 0.0f : _thrust_desired(1));
                _thrust_desired(2) = ( _thrust_desired(2) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(2) < 0.0f) ? 0.0f : _thrust_desired(2));
                _thrust_desired(3) = ( _thrust_desired(3) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(3) < 0.0f) ? 0.0f : _thrust_desired(3));

                float dt_dt = (float)(thrustdesireddata.timestamp - last_timestamp_dt) / 1000000.0f;

                // Estimate desired-thrust derivatives using the original DThrust Kalman filters
                _thrust_desired_dot(0) = thrust_kalman_filter.thrust_kalman_filter_DThrust1(dt_dt, _thrust_desired(0));
                _thrust_desired_dot(1) = thrust_kalman_filter.thrust_kalman_filter_DThrust2(dt_dt, _thrust_desired(1));
                _thrust_desired_dot(2) = thrust_kalman_filter.thrust_kalman_filter_DThrust3(dt_dt, _thrust_desired(2));
                _thrust_desired_dot(3) = thrust_kalman_filter.thrust_kalman_filter_DThrust4(dt_dt, _thrust_desired(3));

                last_timestamp_dt = thrustdesireddata.timestamp;
            }

        }

	if (_param_tfc_use_filtered_thrust.get() > 0.5f) {

                _thrust_measure(0) = thrustdata.thrust_kalman_filter_data_1;
                _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_2;
                _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_3;
                _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_4;

            } else {

                _thrust_measure(0) = thrustdata.thrust_raw_comp[0];
                _thrust_measure(1) = thrustdata.thrust_raw_comp[1];
                _thrust_measure(2) = thrustdata.thrust_raw_comp[2];
                _thrust_measure(3) = thrustdata.thrust_raw_comp[3];
            }

        // Estimate the derivative of the actual thrust measurement selected above.
        // A separate ThrustKalmanFilter object is used so measured- and desired-thrust
        // derivative estimates do not share internal Kalman states.
        if (thrust_measure_updated) {
            thrust_measure_dot[0] = thrust_kalman_filter_measure.thrust_kalman_filter_DThrust1(thrust_measure_dt, _thrust_measure(0));
            thrust_measure_dot[1] = thrust_kalman_filter_measure.thrust_kalman_filter_DThrust2(thrust_measure_dt, _thrust_measure(1));
            thrust_measure_dot[2] = thrust_kalman_filter_measure.thrust_kalman_filter_DThrust3(thrust_measure_dt, _thrust_measure(2));
            thrust_measure_dot[3] = thrust_kalman_filter_measure.thrust_kalman_filter_DThrust4(thrust_measure_dt, _thrust_measure(3));
        }

        /* Thrust closed-loop control using IOLC (Input-Output Linearization Control) */
        // Load feedback weighting coefficients
        _u_fb_coeff(0) = _param_tfc_iolc_k1.get();
        _u_fb_coeff(1) = _param_tfc_iolc_k2.get();
        _u_fb_coeff(2) = _param_tfc_iolc_k3.get();
        _u_fb_coeff(3) = _param_tfc_iolc_k4.get();

        double u_max =
            static_cast<double>(_param_tfc_u_max.get());

        double motor_speed_max =
            static_cast<double>(_param_tfc_motor_speed_max.get());

        if (u_max < 0.0) {
            u_max = 0.0;
        } else if (u_max > 1.0) {
            u_max = 1.0;
        }

        if (motor_speed_max < 100.0) {
            motor_speed_max = 100.0;
        }

        // Use the d_hat retained from the preceding valid LESO update. The
        // current measurement is incorporated after this output is calculated,
        // which preserves causal ordering.
        const float leso_kd = math::constrain(_param_tfc_leso_kd.get(), 0.0f, 1.0f);
        const float leso_d_comp_limit = _param_tfc_leso_dcmax.get();
        const bool leso_compensation_enabled = _param_tfc_leso_en.get() > 0.5f
                                               && leso_kd > 0.0f
                                               && PX4_ISFINITE(leso_d_comp_limit)
                                               && leso_d_comp_limit > 0.0f;
        float leso_d_comp[4] {};

        if (leso_compensation_enabled) {
            for (int i = 0; i < 4; ++i) {
                if (leso[i].initialized && leso_u_applied_valid[i]
                    && PX4_ISFINITE(leso[i].d_hat)) {
                    leso_d_comp[i] = math::constrain(leso[i].d_hat,
                                     -leso_d_comp_limit, leso_d_comp_limit);
                }
            }
        }

        // Motor 1 IOLC parameter assignment and calculation
        _iolc.kp = _param_tfc_iolc_kp1.get();
        _iolc.ki = _param_tfc_iolc_ki1.get();
        _iolc.limit_i = _param_tfc_lim_i.get();
        _iolc.thrust_desired = _thrust_desired(0);
        _iolc_u_ff(0) = _param_tfc_iolc_kff_1.get() * thrustdesireddata.thrust_desired1;
        _iolc.u_ff = _iolc_u_ff(0);
        _iolc.u_fb_coeff = _u_fb_coeff(0);
        _iolc.err = _thrust_desired(0) - _thrust_measure(0);
        _iolc.thrust_desired_dot = _thrust_desired_dot(0);
        _iolc.u_max = u_max;
        _iolc.motor_speed_max = motor_speed_max;
        IOLC_Calculate(&_iolc, leso_kd, leso_d_comp[0]);

        // Motor 2 IOLC parameter assignment and calculation
        _iolc2.kp = _param_tfc_iolc_kp2.get();
        _iolc2.ki = _param_tfc_iolc_ki2.get();
        _iolc2.limit_i = _param_tfc_lim_i.get();
        _iolc2.thrust_desired = _thrust_desired(1);
        _iolc_u_ff(1) = _param_tfc_iolc_kff_2.get() * thrustdesireddata.thrust_desired2;
        _iolc2.u_ff = _iolc_u_ff(1);
        _iolc2.u_fb_coeff = _u_fb_coeff(1);
        _iolc2.err = _thrust_desired(1) - _thrust_measure(1);
        _iolc2.thrust_desired_dot = _thrust_desired_dot(1);
        _iolc2.u_max = u_max;
        _iolc2.motor_speed_max = motor_speed_max;
        IOLC_Calculate2(&_iolc2, leso_kd, leso_d_comp[1]);

        // Motor 3 IOLC parameter assignment and calculation
        _iolc3.kp = _param_tfc_iolc_kp3.get();
        _iolc3.ki = _param_tfc_iolc_ki3.get();
        _iolc3.limit_i = _param_tfc_lim_i.get();
        _iolc3.thrust_desired = _thrust_desired(2);
        _iolc_u_ff(2) = _param_tfc_iolc_kff_3.get() * thrustdesireddata.thrust_desired3;
        _iolc3.u_ff = _iolc_u_ff(2);
        _iolc3.u_fb_coeff = _u_fb_coeff(2);
        _iolc3.err = _thrust_desired(2) - _thrust_measure(2);
        _iolc3.thrust_desired_dot = _thrust_desired_dot(2);
        _iolc3.u_max = u_max;
        _iolc3.motor_speed_max = motor_speed_max;
        IOLC_Calculate3(&_iolc3, leso_kd, leso_d_comp[2]);

        // Motor 4 IOLC parameter assignment and calculation
        _iolc4.kp = _param_tfc_iolc_kp4.get();
        _iolc4.ki = _param_tfc_iolc_ki4.get();
        _iolc4.limit_i = _param_tfc_lim_i.get();
        _iolc4.thrust_desired = _thrust_desired(3);
        _iolc_u_ff(3) = _param_tfc_iolc_kff_4.get() * thrustdesireddata.thrust_desired4;
        _iolc4.u_ff = _iolc_u_ff(3);
        _iolc4.u_fb_coeff = _u_fb_coeff(3);
        _iolc4.err = _thrust_desired(3) - _thrust_measure(3);
        _iolc4.thrust_desired_dot = _thrust_desired_dot(3);
        _iolc4.u_max = u_max;
        _iolc4.motor_speed_max = motor_speed_max;
        IOLC_Calculate4(&_iolc4, leso_kd, leso_d_comp[3]);

        // 新增+2026-09-16：每轮先清除更新标志，保证日志仅标记真正随新测力数据执行的 LESO 更新。
        leso_updated[0] = false;
        leso_updated[1] = false;
        leso_updated[2] = false;
        leso_updated[3] = false;

        // 新增+2026-09-16：读取四路 LESO 共用的默认带宽，单通道覆盖值无效时回退使用该值。
        const float leso_omega_o_default = _param_tfc_leso_wo.get();
        // 新增+2026-09-16：读取电机 1 的带宽覆盖参数。
        const float leso_omega_o_override1 = _param_tfc_leso_wo1.get();
        // 新增+2026-09-16：读取电机 2 的带宽覆盖参数。
        const float leso_omega_o_override2 = _param_tfc_leso_wo2.get();
        // 新增+2026-09-16：读取电机 3 的带宽覆盖参数。
        const float leso_omega_o_override3 = _param_tfc_leso_wo3.get();
        // 新增+2026-09-16：读取电机 4 的带宽覆盖参数。
        const float leso_omega_o_override4 = _param_tfc_leso_wo4.get();
        // 新增+2026-09-16：汇总四路带宽覆盖参数，非正值时表示使用全局默认带宽。
        const float leso_omega_o_override[4] = {
            leso_omega_o_override1,
            leso_omega_o_override2,
            leso_omega_o_override3,
            leso_omega_o_override4
        };

        // 新增+2026-09-16：逐旋翼解析实际生效带宽，支持“全局默认值 + 单通道覆盖”。
        const float leso_omega_o_active[4] = {
            leso_omega_o_override[0] > 0.0f ? leso_omega_o_override[0] : leso_omega_o_default,
            leso_omega_o_override[1] > 0.0f ? leso_omega_o_override[1] : leso_omega_o_default,
            leso_omega_o_override[2] > 0.0f ? leso_omega_o_override[2] : leso_omega_o_default,
            leso_omega_o_override[3] > 0.0f ? leso_omega_o_override[3] : leso_omega_o_default
        };

        // 新增+2026-09-16：LESO 只在新测力样本到达时更新，使用测力时间差而不是 IOLC 内部限幅 dt。
        if (thrust_measure_updated && _param_tfc_leso_en.get() > 0.5f) {
            // 新增+2026-09-16：读取 d_hat 的安全限幅；未配置正值时 LESO_Update 会拒绝更新。
            const float leso_d_state_limit = _param_tfc_leso_dsmax.get();
            // 新增+2026-09-16：通道 1 以当前 f_x/g_x、mixer 实际上一周期输入和新测力样本更新 LESO。
            if (leso_u_applied_valid[0]) {
                leso_updated[0] = LESO_Update(&leso[0], _thrust_measure(0), static_cast<float>(_iolc.f_x),
                                  static_cast<float>(_iolc.g_x), leso_u_applied_prev[0], thrust_measure_dt,
                                  leso_omega_o_active[0], leso_d_state_limit);
            }
            // 新增+2026-09-16：通道 2 以当前 f_x/g_x、mixer 实际上一周期输入和新测力样本更新 LESO。
            if (leso_u_applied_valid[1]) {
                leso_updated[1] = LESO_Update(&leso[1], _thrust_measure(1), static_cast<float>(_iolc2.f_x),
                                  static_cast<float>(_iolc2.g_x), leso_u_applied_prev[1], thrust_measure_dt,
                                  leso_omega_o_active[1], leso_d_state_limit);
            }
            // 新增+2026-09-16：通道 3 以当前 f_x/g_x、mixer 实际上一周期输入和新测力样本更新 LESO。
            if (leso_u_applied_valid[2]) {
                leso_updated[2] = LESO_Update(&leso[2], _thrust_measure(2), static_cast<float>(_iolc3.f_x),
                                  static_cast<float>(_iolc3.g_x), leso_u_applied_prev[2], thrust_measure_dt,
                                  leso_omega_o_active[2], leso_d_state_limit);
            }
            // 新增+2026-09-16：通道 4 以当前 f_x/g_x、mixer 实际上一周期输入和新测力样本更新 LESO。
            if (leso_u_applied_valid[3]) {
                leso_updated[3] = LESO_Update(&leso[3], _thrust_measure(3), static_cast<float>(_iolc4.f_x),
                                  static_cast<float>(_iolc4.g_x), leso_u_applied_prev[3], thrust_measure_dt,
                                  leso_omega_o_active[3], leso_d_state_limit);
            }
        }

    // =========================
    // thrust_leso_data logging
    // =========================
	// 新增+2026-09-16：仅在新测力样本到达时发布 LESO 独立日志，保持与观测器时间轴严格对齐。
	if (thrust_measure_updated) {
	// 新增+2026-09-16：记录本条 LESO 日志生成时刻。
	thrustlesodata.timestamp = hrt_absolute_time();
	// 新增+2026-09-16：记录对应测力样本进入控制器时的时间戳。
	thrustlesodata.timestamp_sample = thrustdata.timestamp;
	// 新增+2026-09-16：记录 LESO 总开关状态。
	thrustlesodata.enabled = static_cast<uint8_t>(_param_tfc_leso_en.get() > 0.5f);
	// 新增+2026-09-16：记录全局默认带宽。
	thrustlesodata.omega_o_default = leso_omega_o_default;
	// 新增+2026-09-16：记录 LESO 扰动状态绝对限幅。
	thrustlesodata.d_state_limit = _param_tfc_leso_dsmax.get();
	// 记录主动补偿是否真正具备生效条件及其参数。
	thrustlesodata.kd_active = leso_kd;
	thrustlesodata.d_comp_limit = leso_d_comp_limit;
	thrustlesodata.compensation_enabled =
	static_cast<uint8_t>(leso_compensation_enabled);
	// 新增+2026-09-16：记录本次观测器使用的真实测力采样间隔。
	thrustlesodata.dt = thrust_measure_dt;
	// 新增+2026-09-16：逐旋翼写入 LESO 状态、输入、更新结果和实际生效参数。
	for (int i = 0; i < 4; ++i) {
		// 新增+2026-09-16：记录该旋翼进入 LESO 的实际升力测量。
		thrustlesodata.thrust_measure[i] = _thrust_measure(i);
		// 新增+2026-09-16：记录内部升力状态估计。
		thrustlesodata.y_hat[i] = leso[i].y_hat;
		// 新增+2026-09-16：记录总升力导数残差估计。
		thrustlesodata.d_hat[i] = leso[i].d_hat;
		// 新增+2026-09-16：记录测量创新量。
		thrustlesodata.innovation[i] = leso[i].innovation;
		// 新增+2026-09-16：记录名义升力导数预测。
		thrustlesodata.nominal_t_dot[i] = leso[i].nominal_t_dot;
		// 新增+2026-09-16：记录内部升力状态导数。
		thrustlesodata.y_hat_dot[i] = leso[i].y_hat_dot;
		// 新增+2026-09-16：记录扰动状态导数。
		thrustlesodata.d_hat_dot[i] = leso[i].d_hat_dot;
		// 新增+2026-09-16：记录 mixer 最终输出对应的上一周期模型输入。
		thrustlesodata.u_applied_prev[i] = leso[i].u_applied_prev;
		// 新增+2026-09-16：记录该通道覆盖带宽参数原值。
		thrustlesodata.omega_o_override[i] = leso_omega_o_override[i];
		// 新增+2026-09-16：记录该旋翼最终实际使用的 LESO 带宽。
		thrustlesodata.omega_o_active[i] = leso_omega_o_active[i];
		// 新增+2026-09-16：记录 mixer 实际输入是否已准备完成。
		thrustlesodata.u_applied_valid[i] = static_cast<uint8_t>(leso_u_applied_valid[i]);
		// 新增+2026-09-16：记录本轮是否完成有效 LESO 更新。
		thrustlesodata.updated[i] = static_cast<uint8_t>(leso_updated[i]);
		// 新增+2026-09-16：记录观测器是否完成首次有效初始化。
		thrustlesodata.initialized[i] = static_cast<uint8_t>(leso[i].initialized);
		// 该 d_comp 是本轮 IOLC_Calculate() 实际使用的补偿量。
		// 它对应当前测力样本更新之前的 LESO 状态，避免非因果地使用未来测量。
		thrustlesodata.d_comp_used[i] = leso_d_comp[i];
		double g_x_used = 0.0;
		switch (i) {
			case 0:
			g_x_used = _iolc.g_x;
			break;

			case 1:
			g_x_used = _iolc2.g_x;
			break;

			case 2:
			g_x_used = _iolc3.g_x;
			break;

			default:
			g_x_used = _iolc4.g_x;
			break;
		}
		// 与 IOLC_ComputeOutput() 中的代数关系一致。
		// 若补偿未使能、g_x 异常或 d_comp 为零，则记录为零。
		if (leso_compensation_enabled && PX4_ISFINITE(g_x_used) && fabs(g_x_used) > 1e-9 && PX4_ISFINITE(leso_d_comp[i])) {
			thrustlesodata.delta_u_leso_unsat[i] = -leso_kd * leso_d_comp[i] / static_cast<float>(g_x_used);
		} else {
			thrustlesodata.delta_u_leso_unsat[i] = 0.0f;
		}
	}

	// 新增+2026-09-16：发布 LESO 独立 uORB 消息，供 logger 写入 ULog。
	thrustlesodata_pub.publish(thrustlesodata);
	}

        // =========================
        // thrust_control_data logging
        // =========================

        // Internal motor-speed derivative estimate
        thrustcontroldata.motor_speed_dot[0] = static_cast<float>(_iolc.motorSpeed_dot);
        thrustcontroldata.motor_speed_dot[1] = static_cast<float>(_iolc2.motorSpeed_dot);
        thrustcontroldata.motor_speed_dot[2] = static_cast<float>(_iolc3.motorSpeed_dot);
        thrustcontroldata.motor_speed_dot[3] = static_cast<float>(_iolc4.motorSpeed_dot);

        // Internal motor-speed estimate
        thrustcontroldata.motor_speed[0] = static_cast<float>(_iolc.motorSpeed);
        thrustcontroldata.motor_speed[1] = static_cast<float>(_iolc2.motorSpeed);
        thrustcontroldata.motor_speed[2] = static_cast<float>(_iolc3.motorSpeed);
        thrustcontroldata.motor_speed[3] = static_cast<float>(_iolc4.motorSpeed);

        // Nonlinear drift term f(x)
        thrustcontroldata.f_x[0] = static_cast<float>(_iolc.f_x);
        thrustcontroldata.f_x[1] = static_cast<float>(_iolc2.f_x);
        thrustcontroldata.f_x[2] = static_cast<float>(_iolc3.f_x);
        thrustcontroldata.f_x[3] = static_cast<float>(_iolc4.f_x);

        // Input gain g(x)
        thrustcontroldata.g_x[0] = static_cast<float>(_iolc.g_x);
        thrustcontroldata.g_x[1] = static_cast<float>(_iolc2.g_x);
        thrustcontroldata.g_x[2] = static_cast<float>(_iolc3.g_x);
        thrustcontroldata.g_x[3] = static_cast<float>(_iolc4.g_x);

        // IOLC calculation time step
        thrustcontroldata.deta_t[0] = static_cast<float>(_iolc.dt);
        thrustcontroldata.deta_t[1] = static_cast<float>(_iolc2.dt);
        thrustcontroldata.deta_t[2] = static_cast<float>(_iolc3.dt);
        thrustcontroldata.deta_t[3] = static_cast<float>(_iolc4.dt);

        // Thrust tracking error: desired - measured
        thrustcontroldata.thrust_error[0] = _iolc.err;
        thrustcontroldata.thrust_error[1] = _iolc2.err;
        thrustcontroldata.thrust_error[2] = _iolc3.err;
        thrustcontroldata.thrust_error[3] = _iolc4.err;

        // Desired thrust
        thrustcontroldata.thrust_desired[0] = _thrust_desired(0);
        thrustcontroldata.thrust_desired[1] = _thrust_desired(1);
        thrustcontroldata.thrust_desired[2] = _thrust_desired(2);
        thrustcontroldata.thrust_desired[3] = _thrust_desired(3);

        // Desired-thrust derivative
        thrustcontroldata.thrust_desired_dot[0] = _thrust_desired_dot(0);
        thrustcontroldata.thrust_desired_dot[1] = _thrust_desired_dot(1);
        thrustcontroldata.thrust_desired_dot[2] = _thrust_desired_dot(2);
        thrustcontroldata.thrust_desired_dot[3] = _thrust_desired_dot(3);

        // Measured-thrust derivative estimated by the independent Kalman filter
        thrustcontroldata.thrust_measure_dot[0] = thrust_measure_dot[0];
        thrustcontroldata.thrust_measure_dot[1] = thrust_measure_dot[1];
        thrustcontroldata.thrust_measure_dot[2] = thrust_measure_dot[2];
        thrustcontroldata.thrust_measure_dot[3] = thrust_measure_dot[3];

        // Weighted IOLC feedback contribution
        thrustcontroldata.thrust_feedback_out[0] =
            _u_fb_coeff(0) * static_cast<float>(_iolc.u_fb);

        thrustcontroldata.thrust_feedback_out[1] =
            _u_fb_coeff(1) * static_cast<float>(_iolc2.u_fb);

        thrustcontroldata.thrust_feedback_out[2] =
            _u_fb_coeff(2) * static_cast<float>(_iolc3.u_fb);

        thrustcontroldata.thrust_feedback_out[3] =
            _u_fb_coeff(3) * static_cast<float>(_iolc4.u_fb);

        // Total IOLC output after feedforward/feedback combination and saturation
        _total_output(0) = static_cast<float>(_iolc.u);
        _total_output(1) = static_cast<float>(_iolc2.u);
        _total_output(2) = static_cast<float>(_iolc3.u);
        _total_output(3) = static_cast<float>(_iolc4.u);

        // Feedforward control contribution
        thrustcontroldata.thrust_feedforward_out[0] = _iolc_u_ff(0);
        thrustcontroldata.thrust_feedforward_out[1] = _iolc_u_ff(1);
        thrustcontroldata.thrust_feedforward_out[2] = _iolc_u_ff(2);
        thrustcontroldata.thrust_feedforward_out[3] = _iolc_u_ff(3);

        // Final normalized control output in [0, u_max]
        thrustcontroldata.thrust_control_out[0] = _total_output(0);
        thrustcontroldata.thrust_control_out[1] = _total_output(1);
        thrustcontroldata.thrust_control_out[2] = _total_output(2);
        thrustcontroldata.thrust_control_out[3] = _total_output(3);

        // Convert normalized control output to PX4-style [-1, 1] signal
        thrustcontroldata.thrust_control_out_pwm[0] =
            math::constrain((2.f * _total_output(0) - 1.f), -1.f, 1.f);

        thrustcontroldata.thrust_control_out_pwm[1] =
            math::constrain((2.f * _total_output(1) - 1.f), -1.f, 1.f);

        thrustcontroldata.thrust_control_out_pwm[2] =
            math::constrain((2.f * _total_output(2) - 1.f), -1.f, 1.f);

        thrustcontroldata.thrust_control_out_pwm[3] =
            math::constrain((2.f * _total_output(3) - 1.f), -1.f, 1.f);

        // Filter/fuse the control output with the open-loop control signal
        _thrust_kalman_filter_control_out(0) =
            thrust_kalman_filter.thrust_kalman_filter_u1(
                thrustcontroldata.thrust_control_out_pwm[0],
                thrustdesireddata.open_loop_control_output1);

        _thrust_kalman_filter_control_out(1) =
            thrust_kalman_filter.thrust_kalman_filter_u2(
                thrustcontroldata.thrust_control_out_pwm[1],
                thrustdesireddata.open_loop_control_output2);

        _thrust_kalman_filter_control_out(2) =
            thrust_kalman_filter.thrust_kalman_filter_u3(
                thrustcontroldata.thrust_control_out_pwm[2],
                thrustdesireddata.open_loop_control_output3);

        _thrust_kalman_filter_control_out(3) =
            thrust_kalman_filter.thrust_kalman_filter_u4(
                thrustcontroldata.thrust_control_out_pwm[3],
                thrustdesireddata.open_loop_control_output4);

        thrustcontroldata.thrust_kalman_filter_control_out_pwm[0] =
            math::constrain(_thrust_kalman_filter_control_out(0), -1.f, 1.f);

        thrustcontroldata.thrust_kalman_filter_control_out_pwm[1] =
            math::constrain(_thrust_kalman_filter_control_out(1), -1.f, 1.f);

        thrustcontroldata.thrust_kalman_filter_control_out_pwm[2] =
            math::constrain(_thrust_kalman_filter_control_out(2), -1.f, 1.f);

        thrustcontroldata.thrust_kalman_filter_control_out_pwm[3] =
            math::constrain(_thrust_kalman_filter_control_out(3), -1.f, 1.f);

        // Integral tracking-error state
        thrustcontroldata.err_integral[0] = _iolc.err_integral;
        thrustcontroldata.err_integral[1] = _iolc2.err_integral;
        thrustcontroldata.err_integral[2] = _iolc3.err_integral;
        thrustcontroldata.err_integral[3] = _iolc4.err_integral;

        // Virtual control v = thrust_desired_dot + Kp*e + Ki*integral(e)
        thrustcontroldata.virtual_control[0] = static_cast<float>(_iolc.v);
        thrustcontroldata.virtual_control[1] = static_cast<float>(_iolc2.v);
        thrustcontroldata.virtual_control[2] = static_cast<float>(_iolc3.v);
        thrustcontroldata.virtual_control[3] = static_cast<float>(_iolc4.v);

        // 各控制量占比记录
        thrustcontroldata.fb_p_part[0] = (_iolc.kp * _iolc.err)/static_cast<float>(_iolc.g_x);
        thrustcontroldata.fb_p_part[1] = (_iolc.kp * _iolc2.err)/static_cast<float>(_iolc2.g_x);
        thrustcontroldata.fb_p_part[2] = (_iolc.kp * _iolc3.err)/static_cast<float>(_iolc3.g_x);
        thrustcontroldata.fb_p_part[3] = (_iolc.kp * _iolc4.err)/static_cast<float>(_iolc4.g_x);

        thrustcontroldata.fb_i_part[0] = (_iolc.ki * _iolc.err_integral)/static_cast<float>(_iolc.g_x);
        thrustcontroldata.fb_i_part[1] = (_iolc.ki * _iolc2.err_integral)/static_cast<float>(_iolc2.g_x);
        thrustcontroldata.fb_i_part[2] = (_iolc.ki * _iolc3.err_integral)/static_cast<float>(_iolc3.g_x);
        thrustcontroldata.fb_i_part[3] = (_iolc.ki * _iolc4.err_integral)/static_cast<float>(_iolc4.g_x);

        thrustcontroldata.fb_nolin_part[0] = static_cast<float>(_iolc.f_x) / static_cast<float>(_iolc.g_x);
        thrustcontroldata.fb_nolin_part[1] = static_cast<float>(_iolc2.f_x) / static_cast<float>(_iolc2.g_x);
        thrustcontroldata.fb_nolin_part[2] = static_cast<float>(_iolc3.f_x) / static_cast<float>(_iolc3.g_x);
        thrustcontroldata.fb_nolin_part[3] = static_cast<float>(_iolc4.f_x) / static_cast<float>(_iolc4.g_x);

        // Scalar state/selection values for log analysis
        thrustcontroldata.thrust_start = _param_tfc_start.get();
        // Existing `kd` log field records the active LESO compensation gain.
        // thrustcontroldata.kd = leso_kd;
        // thrustcontroldata.sensor_model = _param_tfc_sensor_model.get();

        // Publish one thrust_control_data sample
        thrustcontroldata.timestamp = hrt_absolute_time();
        _thrustcontroldata_pub.publish(thrustcontroldata);

        px4_usleep(1000);

    }

    return 0;
}
