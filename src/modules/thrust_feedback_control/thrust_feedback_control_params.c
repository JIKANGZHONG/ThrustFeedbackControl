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
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_1, 3.5f);

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
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_2, 3.5f);

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
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_3, 3.5f);

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
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_4, 3.5f);

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
 * Sensor1 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR1_BIAS, 0.0f);

/**
 * Sensor2 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR2_BIAS, 0.0f);

/**
 * Sensor3 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR3_BIAS, 0.0f);

/**
 * Sensor4 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR4_BIAS, 0.0f);

/**
 * Use filtered thrust data or not
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(THR_USE_FIL, 0.0f);

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
