#include "chassis_speed.h"
#include <stdlib.h>

// X 型麦轮运动学逆解（以底盘中心为原点，X向前，Y向左）
// 电机编号：0-左前，1-左后，2-右后，3-右前
void Mecanum_Calculate(int16_t vx, int16_t vy, int16_t vz, int16_t motor_targets[4])
{
    // 加入死区处理（消除摇杆回中后的小幅度漂移）
#define DEADBAND 15
    if (abs(vx) < DEADBAND) vx = 0;
    if (abs(vy) < DEADBAND) vy = 0;
    if (abs(vz) < DEADBAND) vz = 0;

    // 将摇杆值映射到 RPM（带缩放）
    float vx_f = (float)vx * SPEED_SCALE;
    float vy_f = (float)vy * SPEED_SCALE;
    float vz_f = (float)vz * SPEED_SCALE;   // 旋转系数，可根据需要调整 (例如 *1.0)

    // M1(左前) = -Vx + Vy +Vz
    // M2(左后) = -Vx - Vy + Vz
    // M3(右后) = Vx - Vy + Vz
    // M4(右前) = Vx + Vy + Vz
    float w1 = vx_f + vy_f + vz_f;
    float w2 = vx_f - vy_f + vz_f;
    float w3 = -vx_f - vy_f + vz_f;
    float w4 = -vx_f + vy_f + vz_f;

    // 限幅到最大转速（防止超调，例如 ±3000 RPM）
#define MAX_RPM 10000

    if (w1 > MAX_RPM) {w1 = MAX_RPM;}
    if (w1 < -MAX_RPM){w1 = -MAX_RPM;}
    if (w2 > MAX_RPM) {w2 = MAX_RPM;}
    if (w2 < -MAX_RPM) {w2 = -MAX_RPM;}
    if (w3 > MAX_RPM) {w3 = MAX_RPM;}
    if (w3 < -MAX_RPM) {w3 = -MAX_RPM;}
    if (w4 > MAX_RPM) {w4 = MAX_RPM;}
    if (w4 < -MAX_RPM) {w4 = -MAX_RPM;}

    motor_targets[0] = (int16_t)w1;   // 左前
    motor_targets[1] = (int16_t)w2;   // 左后
    motor_targets[2] = (int16_t)w3;   // 右后
    motor_targets[3] = (int16_t)w4;   // 右前
}

