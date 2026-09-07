//
// Created by LENOVO on 2026/7/27.
//
#include "pid.h"


void PID_Init(PID_t *pid, const PID_Init_Config_t pid_init_config) {
    pid->Kp = pid_init_config.Kp;
    pid->Ki = pid_init_config.Ki;
    pid->Kd = pid_init_config.Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_max = pid_init_config.output_max;
    pid->integral_max = pid_init_config.integral_max;
}
float PID_Calculate(PID_t *pid, float target, float actual) {
    float error = target - actual;
    // 积分累加（带限幅防饱和）
    pid->integral += error;
    float output_integral = pid->Ki * pid->integral;
    if (output_integral > pid->integral_max) output_integral = pid->integral_max;
    if (output_integral < -pid->integral_max) output_integral = -pid->integral_max;

    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = pid->Kp * error + output_integral + pid->Kd * derivative;
    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    return output;
}
