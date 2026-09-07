#ifndef __CHASSIS_SPEED_H
#define __CHASSIS_SPEED_H

#include "stdint.h"

// 麦轮底盘类型定义
#define MECANUM_X_TYPE  1   // X 型（常用）
#define MECANUM_O_TYPE  0   // O 型

// 转速缩放系数（将摇杆量 -660~+660 映射到 RPM）
#define SPEED_SCALE     6.6f   // 例如：摇杆 660 -> 1188 RPM

// 输入：vx(前后), vy(左右), vz(自旋)，范围通常 -660 ~ +660
// 输出：4 个电机目标转速 (RPM)，写入 motor_targets[0]~[3]
void Mecanum_Calculate(int16_t vx, int16_t vy, int16_t vz, int16_t motor_targets[4]);

#endif
