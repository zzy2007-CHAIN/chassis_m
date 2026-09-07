//
// Created by LENOVO on 2026/7/27.
//
#include "main.h"
#include "Mycan.h"
#include "3508.h"
#include "can.h"

// 定义电调CAN ID
#define MOTOR1_CAN_ID  0x201
#define MOTOR2_CAN_ID  0x202
#define MOTOR3_CAN_ID  0x203
#define MOTOR4_CAN_ID  0x204
#define MOTOR_CURRENT_CMD_LIMIT  6000
/* USER CODE END 0 */

static int16_t CAN_LimitCurrentCommand(int16_t current)
{
  if (current > MOTOR_CURRENT_CMD_LIMIT) {
    return MOTOR_CURRENT_CMD_LIMIT;
  }
  if (current < -MOTOR_CURRENT_CMD_LIMIT) {
    return -MOTOR_CURRENT_CMD_LIMIT;
  }
  return current;
}

/* CAN init function */
void MX_CAN_Init(void)
{
// 过滤器配置
  CAN_FilterTypeDef can_filter = {0};

  can_filter.FilterBank = 0;                     // CAN2使用过滤器组14
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // 关联到FIFO0
  can_filter.FilterActivation = ENABLE;           // 激活过滤器
  can_filter.SlaveStartFilterBank = 0;            // 0~13分配给CAN1，14~27分配给CAN2

  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  // 掩码全0：接收全部报文，再在CAN_RxCallback中筛选0x201~0x204
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;

  if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
    Error_Handler();
  }
  // if (HAL_CAN_Start(&hcan1) != HAL_OK) {
  //   Error_Handler();
  // }
  // if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  //   Error_Handler();
  // }
  HAL_CAN_Start(&hcan1);                      // 启动 CAN
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能 CAN 内部中断标志
}


void CAN_Send_Current(int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4) {
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t txMailbox;

  cur1 = CAN_LimitCurrentCommand(cur1);
  cur2 = CAN_LimitCurrentCommand(cur2);
  cur3 = CAN_LimitCurrentCommand(cur3);
  cur4 = CAN_LimitCurrentCommand(cur4);

  txHeader.StdId = 0x200;               // C620控制指令广播ID
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  // 大端模式填充（高字节在前）
  txData[0] = (uint8_t)(cur1 >> 8 & 0xFF);
  txData[1] = (uint8_t)(cur1 & 0xFF);
  txData[2] = (uint8_t)(cur2 >> 8 & 0xFF);
  txData[3] = (uint8_t)(cur2 & 0xFF);
  txData[4] = (uint8_t)(cur3 >> 8 & 0xFF);
  txData[5] = (uint8_t)(cur3 & 0xFF);
  txData[6] = (uint8_t)(cur4 >> 8 & 0xFF);
  txData[7] = (uint8_t)(cur4 & 0xFF);


  HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
  /* Toggle debug LED to indicate CAN send */
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

// CAN数据解析函数（由中断回调调用）
void CAN_RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData) {
  uint8_t motor_index;
  // 1. 判断ID是否在 0x201~0x204
  if (rxHeader->StdId < MOTOR1_CAN_ID || rxHeader->StdId > MOTOR4_CAN_ID) {
    return;
  }
  // 2. 计算电机索引
  motor_index = rxHeader->StdId - MOTOR1_CAN_ID;
  // 判断是否为我们配置的电调ID（MOTOR_CAN_ID）
    g_motor_ctrl.Motor[motor_index].motor_feedback.pre_angle      = g_motor_ctrl.Motor[motor_index].motor_feedback.cur_angle;
    g_motor_ctrl.Motor[motor_index].motor_feedback.cur_angle      = (float)((int16_t)(rxData[0] << 8 | rxData[1]) / 8191.f * 360.f);
    // 圈数累加器（基于角度值检测回绕）
    float delta = g_motor_ctrl.Motor[motor_index].motor_feedback.cur_angle - g_motor_ctrl.Motor[motor_index].motor_feedback.pre_angle;
    if (delta > 180.0f) {           // 正向越过0°（如 350°→10°）
      g_motor_ctrl.Motor[motor_index].motor_feedback.cnt --;
    } else if (delta < -180.0f) {   // 反向越过0°（如 10°→350°）
      g_motor_ctrl.Motor[motor_index].motor_feedback.cnt ++;
    }
    // 计算当前累计角度（圈数 × 360° + 当前角度）
    g_motor_ctrl.Motor[motor_index].motor_feedback.total_angle =
        (float)g_motor_ctrl.Motor[motor_index].motor_feedback.cnt * 360.0f + g_motor_ctrl.Motor[motor_index].motor_feedback.cur_angle;
    g_motor_ctrl.Motor[motor_index].motor_feedback.total_P19_angle = g_motor_ctrl.Motor[motor_index].motor_feedback.total_angle / 19.0f;
    g_motor_ctrl.Motor[motor_index].motor_feedback.speed_rpm  = (int16_t)(rxData[2] << 8 | rxData[3]);
    g_motor_ctrl.Motor[motor_index].motor_feedback.current    = (int16_t)(rxData[4] << 8 | rxData[5]);
    g_motor_ctrl.Motor[motor_index].motor_feedback.temp       = (int8_t)(rxData[6]);
}
