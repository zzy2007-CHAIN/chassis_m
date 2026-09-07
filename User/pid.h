//
// Created by LENOVO on 2026/7/27.
//

#ifndef CHASSIS_PID_H
#define CHASSIS_PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_max;   // 输出绝对值上限
    float integral_max; // 积分绝对值上限
    float target;
} PID_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float output_max;   // 输出绝对值上限
    float integral_max; // 积分绝对值上限
} PID_Init_Config_t;


extern PID_t pid1, pid2, pid3, pid4;
extern PID_t pid_config;

void PID_Init(PID_t *pid, const PID_Init_Config_t pid_init_config);
float PID_Calculate(PID_t *pid, float target, float actual);

#endif //CHASSIS_PID_H
