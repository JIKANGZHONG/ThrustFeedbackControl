/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * The MAX_VALUE of the Force
 *
 * force_feedback the max_value of the force.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_THRUST_MAX, 2.0f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 1.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_1, 2.312f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 2.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_2, 1.954f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 3.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_3, 2.544f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 4.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_4, 2.517f);

/**
 * IOLC Coefficient K
 *
 * Input-Output Linearization Controller Gain K.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K, 1.0f);

/**
 * IOLC Coefficient ki
 *
 * Input-Output Linearization Controller Gain ki.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KI, 0.1f);

/**
 * The Limit value of I_out
 *
 * The limit value of the PID controller I_out, i.e. I_out=[-limit,limit].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LIM_I, 1.0f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_1, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_2, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_3, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_4, 0.6f);

/**
 * Kalman Filter Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of estimation. i.e. the standard deviation of the acceleration of the force.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_SD_ES, 5.0f);

/**
 * Kalman Filter Standard deviation of measurement
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_SD_ME, 0.05f);

/**
 * Kalman Filter Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of estimation. i.e. the standard deviation of the acceleration of the force.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_TD_ES, 5.0f);

/**
 * Kalman Filter Standard deviation of measurement
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_TD_ME, 0.05f);

/**
 * The correction factor of control output
 *
 * Force_d[0~1] = (1-alpha)*omega_d + alpha*omega_d^2, omega is the output of the controller[0~1].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_ALPHA, 0.0f);

/**
 * IOLC Coefficient kp1
 *
 * Input-Output Linearization Controller Gain kp1.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP1, 0.8f);

/**
 * IOLC Coefficient kp2
 *
 * Input-Output Linearization Controller Gain kp2.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP2, 0.8f);

/**
 * IOLC Coefficient kp3
 *
 * Input-Output Linearization Controller Gain kp3.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP3, 0.8f);

/**
 * IOLC Coefficient kp4
 *
 * Input-Output Linearization Controller Gain kp4.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP4, 0.8f);

/**
 * IOLC Coefficient ki1
 *
 * Input-Output Linearization Controller Gain ki1.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KI1, 0.1f);

/**
 * IOLC Coefficient ki2
 *
 * Input-Output Linearization Controller Gain ki2.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KI2, 0.1f);

/**
 * IOLC Coefficient ki3
 *
 * Input-Output Linearization Controller Gain ki3.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KI3, 0.1f);

/**
 * IOLC Coefficient ki4
 *
 * Input-Output Linearization Controller Gain ki4.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KI4, 0.1f);

/**
 * IOLC Coefficient K1
 *
 * Input-Output Linearization Controller Gain K1.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K1, 1.0f);

/**
 * IOLC Coefficient K2
 *
 * Input-Output Linearization Controller Gain K2.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K2, 1.0f);

/**
 * IOLC Coefficient K3
 *
 * Input-Output Linearization Controller Gain K3.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K3, 1.0f);

/**
 * IOLC Coefficient K4
 *
 * Input-Output Linearization Controller Gain K4.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K4, 1.0f);

/**
 * Mode Switching Coefficient
 *
 * thrust open-loop and thrust closed-loop and thrust output Kalman filter control.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 0.01
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_START, 0.0f);

/**
 * Kalman Filter Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of estimation. i.e. the standard deviation of the acceleration of the force.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_ES_U, 0.1f);

/**
 * Kalman Filter Standard deviation of measurement
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_ME_U, 0.1f);

/**
 * Motorspeed Desired
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_MS_FF, 50.0f);

/**
 * PID Coefficient kp
 *
 * Input-Output Linearization Controller Gain kp.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_KP, 0.05f);

/**
 * PID Coefficient ki
 *
 * Input-Output Linearization Controller Gain ki.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_KI, 0.015f);

/**
 * PID Coefficient kd
 *
 * Input-Output Linearization Controller Gain kd.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_KD, 0.003f);

/**
 * The Limit value of I_out
 *
 * The limit value of the PID controller I_out, i.e. I_out=[-limit,limit].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_LIM_I, 1.0f);

/**
 * The Base_value of the i_factor
 *
 * Ki = Ki * i_factor, where i_factor = force_error / FORCECTL_FAC_I,
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_FAC_I, 0.7f);

/**
 * Sensor1 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR1_BIAS1, 260.0f);
// /**
//  * Sensor1 Bias2
//  *
//  * @group Thrust Feedback Control
//  */
// PARAM_DEFINE_FLOAT(SENSOR1_BIAS2, -635.0f);
/**
 * Sensor2 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR2_BIAS1, 0.0f);
// /**
//  * Sensor2 Bias2
//  *
//  * @group Thrust Feedback Control
//  */
// PARAM_DEFINE_FLOAT(SENSOR2_BIAS2, -785.0f);

/**
 * Sensor3 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR3_BIAS1, 156.0f);
// /**
//  * Sensor3 Bias2
//  *
//  * @group Thrust Feedback Control
//  */
// PARAM_DEFINE_FLOAT(SENSOR3_BIAS2, -266.0f);
/**
 * Sensor4 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR4_BIAS1, 261.0f);
// /**
//  * Sensor4 Bias2
//  *
//  * @group Thrust Feedback Control
//  */
// PARAM_DEFINE_FLOAT(SENSOR4_BIAS2, -465.0f);

/**
 * Use filtered thrust data or not
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(THR_USE_FIL, 0.0f);
/**
 * The AlphaFilter parameter
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(ALPHA_TAU, 0.25f);
/**
 * Maximum motor speed limit (RPM)
 *
 * The upper bound for the internal motor speed estimate in the IOLC model.
 *
 * @group Thrust Feedback Control
 * @min 100
 * @max 6000
 * @decimal 1
 * @increment 100
 */
PARAM_DEFINE_FLOAT(MOTOR_SPEED_MAX, 5000.0f);
/**
 * Maximum control output limit (normalized)
 *
 * Upper saturation value for the IOLC output 'u' before PWM conversion.
 * Typical range is 0.0~1.0, but original hardcoded value was 0.5.
 *
 * @group Thrust Feedback Control
 * @min 0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(TFC_U_MAX, 1f);
// 新增+2026-09-18：g_x 积分坐标重标定总开关。
/**
 * IOLC integral g_x rescaling enable
 *
 * Set to 1 to preserve the input-domain integral contribution when g_x changes.
 * Set to 0 to retain the original integral update.
 *
 * @group Thrust Feedback Control
 * @min 0
 * @max 1
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_FLOAT(TFC_IRES_EN, 0.0f);

// 新增+2026-09-18：重标定有效 g_x 下限。
/**
 * IOLC integral rescaling minimum g_x
 *
 * Non-positive values use the implementation minimum positive value (1e-9).
 *
 * @group Thrust Feedback Control
 * @min 0
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(TFC_IRES_GMIN, 0.0f);

// 新增+2026-09-18：重标定比例下限接口，默认关闭。
/**
 * IOLC integral rescaling ratio minimum
 *
 * Non-positive value disables this lower ratio limit.
 *
 * @group Thrust Feedback Control
 * @min 0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(TFC_IRES_RMIN, 0.0f);

// 新增+2026-09-18：重标定比例上限接口，默认关闭。
/**
 * IOLC integral rescaling ratio maximum
 *
 * Non-positive value disables this upper ratio limit.
 *
 * @group Thrust Feedback Control
 * @min 0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(TFC_IRES_RMAX, 0.0f);
// 新增+2026-09-16：LESO 观测器总开关参数定义。
/**
 * LESO observer enable
 *
 * Enable the observer-only LESO. This parameter does not enable
 * disturbance compensation in the current implementation.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_EN, 0.0f);

// 新增+2026-09-16：LESO 全局默认带宽参数定义。
/**
 * LESO default observer bandwidth
 *
 * Default observer bandwidth omega_o used by all motors without an
 * individual positive override. A conservative initial value for the
 * current approximately 100 Hz thrust measurement is 45 rad/s.
 *
 * @unit rad/s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_WO, 45.0f);

// 新增+2026-09-16：电机 1 LESO 带宽覆盖参数定义。
/**
 * LESO bandwidth override motor 1
 *
 * Positive value overrides TFC_LESO_WO for motor 1.
 * Zero means use TFC_LESO_WO.
 *
 * @unit rad/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_WO1, 45.0f);

// 新增+2026-09-16：电机 2 LESO 带宽覆盖参数定义。
/**
 * LESO bandwidth override motor 2
 *
 * Positive value overrides TFC_LESO_WO for motor 2.
 * Zero means use TFC_LESO_WO.
 *
 * @unit rad/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_WO2, 45.0f);

// 新增+2026-09-16：电机 3 LESO 带宽覆盖参数定义。
/**
 * LESO bandwidth override motor 3
 *
 * Positive value overrides TFC_LESO_WO for motor 3.
 * Zero means use TFC_LESO_WO.
 *
 * @unit rad/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_WO3, 45.0f);

// 新增+2026-09-16：电机 4 LESO 带宽覆盖参数定义。
/**
 * LESO bandwidth override motor 4
 *
 * Positive value overrides TFC_LESO_WO for motor 4.
 * Zero means use TFC_LESO_WO.
 *
 * @unit rad/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_WO4, 45.0f);

// 新增+2026-09-16：LESO 扰动状态限幅参数定义。
/**
 * LESO disturbance-state limit
 *
 * Absolute limit of d_hat. Its unit is the same as the thrust-derivative
 * unit used by the current TFC model. A value of zero disables LESO update
 * in the current implementation.
 *
 * @min 1.0
 * @max 1000.0
 * @decimal 1
 * @increment 10.0
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LESO_DSMAX, 200.0f);

// 新增+2026-09-16：LESO 主动补偿增益参数定义。
/**
 * LESO active-compensation gain k_d
 *
 * Zero preserves observer-only behaviour. Increase only through controlled
 * ground testing after the observer and input chain have been verified.
 *
 * @group Thrust Feedback Control
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(TFC_LESO_KD, 0.0f);

// 新增+2026-09-16：LESO 补偿限幅参数定义。
/**
 * LESO compensation limit (N/s)
 *
 * This limit is applied to d_hat before it enters the control law. A
 * non-positive value disables active compensation even if TFC_LESO_KD > 0.
 *
 * @group Thrust Feedback Control
 * @min 0
 * @max 200
 * @decimal 1
 * @increment 1
 */
PARAM_DEFINE_FLOAT(TFC_LESO_DCMAX, 0.0f);
