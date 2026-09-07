/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Mycan.h"
#include "3508.h"
#include "chassis_speed.h"
#include "pid.h"
#include "remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t remote_control_enabled = 0;  // 0=急停锁定, 1=可控制

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_CAN_Init();
  Motor_Control_Init();
  DR16_Init(&huart3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    CAN_Send_Current(3000, 0, 0, 0);  // 发送 0 电流，确保电机不转
    HAL_Delay(10);  // 10ms 控制周期

    // // 1. 仅在有新遥控数据时处理
    // if (DR16_HasNewData()) {
    //   RC_Ctl_t *rc = DR16_GetData();
    //   DR16_ClearDataFlag();
    //
    //   uint8_t s1_state = rc->rc.s1;  // 注意：你的结构体里是 rc.s1
    //   uint8_t s2_state = rc->rc.s2;
    //   // 情况 A：S1 和 S2 同时拨到下方 (DOWN=2) —— 强制急停
    //   if (s1_state == RC_SW_DOWN && s2_state == RC_SW_DOWN) {
    //     remote_control_enabled = 0;   // 锁定控制
    //     continue;
    //   }
    //   // 情况 B：S1 和 S2 同时拨到中间 (MID=3) —— 解锁控制
    //   if (s1_state == RC_SW_MID && s2_state == RC_SW_MID) {
    //     remote_control_enabled = 1;
    //
    //   }
    //   // if (!remote_control_enabled) {
    //   //   // 直接停转（发送 0 电流指令，并清除 PID 积分）
    //   //   int16_t zero_rpm[4] = {0, 0, 0, 0};
    //   //   float zero_target[4] = {0, 0, 0, 0};
    //   //
    //   //   // 清空 PID 积分项，防止解除急停时突然猛冲
    //   //   PID_SetTarget(&g_motor_ctrl.Motor[0].speed_pid, 0);
    //   //   PID_SetTarget(&g_motor_ctrl.Motor[1].speed_pid, 0);
    //   //   PID_SetTarget(&g_motor_ctrl.Motor[2].speed_pid, 0);
    //   //   PID_SetTarget(&g_motor_ctrl.Motor[3].speed_pid, 0);
    //   //
    //   //   // 发送 0 电流
    //   //   Motor_Control_Update(zero_target);
    //   //
    //   //   // 跳过后面的摇杆处理
    //   //   continue;
    //   // }
    //   //2. 提取摇杆通道值（范围 364~1684，减去 1024 得到 -660~+660）
    //   int16_t vx = rc->rc.ch1 - RC_CH_VALUE_OFFSET;  // forward/back
    //   int16_t vy = rc->rc.ch0 - RC_CH_VALUE_OFFSET;  // left/right
    //   int16_t vz = rc->rc.ch2 - RC_CH_VALUE_OFFSET;  // rotate
    //
    //   // 3. 运动学解算：得到 4 个电机的目标转速 (RPM)
    //   int16_t motor_rpm[4];
    //   Mecanum_Calculate(vx, vy, vz, motor_rpm);
    //   // 4. 设置电机目标转速
    //   PID_SetTarget(&g_motor_ctrl.Motor[0].speed_pid, motor_rpm[0]);
    //   PID_SetTarget(&g_motor_ctrl.Motor[1].speed_pid, motor_rpm[1]);
    //   PID_SetTarget(&g_motor_ctrl.Motor[2].speed_pid, motor_rpm[2]);
    //   PID_SetTarget(&g_motor_ctrl.Motor[3].speed_pid, motor_rpm[3]);
    //   float motor_target_float[4] = {
    //     (float)motor_rpm[0],
    //     (float)motor_rpm[1],
    //     (float)motor_rpm[2],
    //     (float)motor_rpm[3]
    // };
    //   Motor_Control_Update(motor_target_float);
    // }
    // HAL_Delay(10);  // 5ms 控制周期
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        CAN_RxCallback(&rxHeader, rxData); // 转交给驱动层解析
      }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
