//
// Created by LENOVO on 2026/7/27.
//

#ifndef CHASSIS_3508_H
#define CHASSIS_3508_H

#include "pid.h"
#include "Mycan.h"

typedef struct {
    PID_t speed_pid;        // 速度环PID控制器
    PID_t angle_pid;            // 位置环PID控制器
    float target_speed; // 目标转速 (RPM)
    float current_output; // 当前输出电流 (单位：mA)
    float target_angle_deg;     // 目标角度（单位：度）
    float current_total_angle_deg;
    Motor_Feedback_t motor_feedback;
} Motor_Control_t;

typedef struct {
    Motor_Control_t Motor[4];
}Motor;

extern Motor g_motor_ctrl;

void Motor_Control_Init(void);
void PID_SetTarget(PID_t *pid, float target);
void Motor_Control_Update(float target_speed[4]);


#endif //CHASSIS_3508_H

