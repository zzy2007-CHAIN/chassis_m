//
// Created by LENOVO on 2026/7/27.
//

#ifndef CHASSIS_MYCAN_H
#define CHASSIS_MYCAN_H

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
typedef struct {
    int16_t speed_rpm;   // 转速 (RPM)
    float pre_angle;
    float cur_angle;       // 机械角度 (0~8192对应0~360°)
    float total_angle;
    float total_P19_angle;
    int16_t current;     // 实际电流
    int16_t cnt;
    int16_t temp;
} Motor_Feedback_t;


void MX_CAN_Init(void);
void CAN_Send_Current(int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);
void CAN_RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData); // 由HAL回调调用


#endif //CHASSIS_MYCAN_H
