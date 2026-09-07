//
// Created by LENOVO on 2026/7/27.
//
#include "pid.h"
#include "Mycan.h"   // 使用 g_motor_fb 和 CAN_Send_Current
#include "3508.h"
#include "chassis_speed.h"
// 定义全局控制对象
Motor g_motor_ctrl={0};
// PID_Init_Config_t angle_pid_config = {
//     .Kp = 5.f,
//     .Ki = 0.0f,
//     .Kd = 50.0f,
//     .output_max = 200.0f,
//     .integral_max = 10.0f
// };
static const PID_Init_Config_t speed_pid_config[4] = {
    {10.0f, 0.0f, 0.0f, 10000.0f, 1200.0f},  // 电机1
    {10.0f, 0.0f, 0.0f, 10000.0f, 1200.0f},  // 电机2
    {10.0f, 0.0f, 0.0f, 10000.0f, 1200.0f},  // 电机3
    {10.0f, 0.0f, 0.0f, 10000.0f, 1200.0f}   // 电机4
};


void Motor_Control_Init(void) {
    for (int i = 0; i < 4; i++) {
        PID_Init(&g_motor_ctrl.Motor[i].speed_pid, speed_pid_config[i]);
        // PID_Init(&g_motor_ctrl.Motor[i].angle_pid,angle_pid_config);
        // 初始化变量
        g_motor_ctrl.Motor[i].target_angle_deg = 0.0f;
        g_motor_ctrl.Motor[i].current_total_angle_deg = 0.0f;
        g_motor_ctrl.Motor[i].motor_feedback.cur_angle = 0.0f;
        g_motor_ctrl.Motor[i].motor_feedback.total_P19_angle = 0.0f;
        g_motor_ctrl.Motor[i].motor_feedback.cnt = 0;
        g_motor_ctrl.Motor[i].current_output = 0;
    }
}

void Motor_Control_Update(float target_speed[4]) {
    float current_cmd[4];
    for (int i = 0; i < 4; i++) {
        // 2. 更新目标转速
        g_motor_ctrl.Motor[i].target_speed = target_speed[i];
        // 3. 调用 PID 计算
        current_cmd[i] = PID_Calculate(
            &g_motor_ctrl.Motor[i].speed_pid,
            g_motor_ctrl.Motor[i].target_speed,
            g_motor_ctrl.Motor[i].motor_feedback.speed_rpm
        );

        // 4. 保存输出
        g_motor_ctrl.Motor[i].current_output = current_cmd[i];
    }

    // 5. 发送 CAN 指令
    CAN_Send_Current(
        (int16_t)current_cmd[0],
        (int16_t)current_cmd[1],
        (int16_t)current_cmd[2],
        (int16_t)current_cmd[3]
    );
}
void PID_SetTarget(PID_t *pid, float target) {
    pid->target = target;  // 就是把目标值存进结构体
}

