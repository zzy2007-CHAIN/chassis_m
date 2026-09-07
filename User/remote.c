#include "remote.h"
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usart.h"

static UART_HandleTypeDef *p_uart = NULL;
static uint8_t rx_buffer[RC_FRAME_LENGTH] = {0};
static RC_Ctl_t rc_ctrl_data = {0};
volatile uint16_t dr16_rx_size = 0;
volatile uint32_t dr16_frame_count = 0;
volatile uint32_t dr16_drop_count = 0;
volatile uint8_t dr16_raw_data[RC_FRAME_LENGTH] = {0};
volatile uint8_t dr16_valid = 0;
volatile uint16_t dr16_ch0 = 0;
volatile uint16_t dr16_ch1 = 0;
volatile uint16_t dr16_ch2 = 0;
volatile uint16_t dr16_ch3 = 0;
static volatile uint8_t data_ready = 0;   // 新数据标志

// 内部解析函数（修正了标准库版本的拨杆和键盘解析错误）
static void DR16_ParseData(uint8_t *pData)
{
    if (pData == NULL) return;
    for (uint8_t i = 0; i < RC_FRAME_LENGTH; i++) {
        dr16_raw_data[i] = pData[i];
    }

    // 四个通道值
    rc_ctrl_data.rc.ch0 = ((uint16_t)pData[0] | ((uint16_t)pData[1] << 8)) & 0x07FF;
    rc_ctrl_data.rc.ch1 = (((uint16_t)pData[1] >> 3) | ((uint16_t)pData[2] << 5)) & 0x07FF;
    rc_ctrl_data.rc.ch2 = (((uint16_t)pData[2] >> 6) | ((uint16_t)pData[3] << 2) | ((uint16_t)pData[4] << 10)) & 0x07FF;
    rc_ctrl_data.rc.ch3 = (((uint16_t)pData[4] >> 1) | ((uint16_t)pData[5] << 7)) & 0x07FF;
    dr16_ch0 = rc_ctrl_data.rc.ch0;
    dr16_ch1 = rc_ctrl_data.rc.ch1;
    dr16_ch2 = rc_ctrl_data.rc.ch2;
    dr16_ch3 = rc_ctrl_data.rc.ch3;

    // 【修正】拨杆 S1 / S2（原代码移位错误）
    rc_ctrl_data.rc.s1 = (pData[5] >> 6) & 0x03;
    rc_ctrl_data.rc.s2 = (pData[5] >> 4) & 0x03;

    // 鼠标（虽然 DR16 没有鼠标，保留兼容）
    rc_ctrl_data.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    rc_ctrl_data.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    rc_ctrl_data.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    rc_ctrl_data.mouse.press_l = pData[12];
    rc_ctrl_data.mouse.press_r = pData[13];

    // 【修正】键盘完整 16 位（原代码只取了低 8 位）
    rc_ctrl_data.key.v = ((uint16_t)pData[14]) | ((uint16_t)pData[15] << 8);
}

static void DR16_StartReceive(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(p_uart, rx_buffer, RC_FRAME_LENGTH);
    __HAL_DMA_DISABLE_IT(p_uart->hdmarx, DMA_IT_HT);
}

static uint8_t DR16_DataIsValid(void)
{
    dr16_valid =
        rc_ctrl_data.rc.ch0 >= RC_CH_VALUE_MIN && rc_ctrl_data.rc.ch0 <= RC_CH_VALUE_MAX &&
        rc_ctrl_data.rc.ch1 >= RC_CH_VALUE_MIN && rc_ctrl_data.rc.ch1 <= RC_CH_VALUE_MAX &&
        rc_ctrl_data.rc.ch2 >= RC_CH_VALUE_MIN && rc_ctrl_data.rc.ch2 <= RC_CH_VALUE_MAX &&
        rc_ctrl_data.rc.ch3 >= RC_CH_VALUE_MIN && rc_ctrl_data.rc.ch3 <= RC_CH_VALUE_MAX;

    return dr16_valid;
}

// 初始化：保存句柄，启动 DMA 循环接收
void DR16_Init(UART_HandleTypeDef *huart3)
{
    p_uart = huart3;
    memset(&rc_ctrl_data, 0, sizeof(RC_Ctl_t));
    data_ready = 0;

    DR16_StartReceive();
}

// 获取最新遥控器数据（主循环调用）
RC_Ctl_t *DR16_GetData(void)
{
    return &rc_ctrl_data;
}

// UART 空闲中断回调（由 HAL 库自动调用）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != p_uart) {
        return;
    }

    dr16_rx_size = Size;
    if (Size == RC_FRAME_LENGTH) {
        DR16_ParseData(rx_buffer);
        if (DR16_DataIsValid()) {
            data_ready = 1;
            dr16_frame_count++;
        } else {
            dr16_drop_count++;
        }
        // DMA 循环模式会自动继续接收，无需重启
    } else {
        dr16_drop_count++;
    }

    DR16_StartReceive();
}

// Called from ISR when IDLE detected
void DR16_IdleIRQHandler(void)
{
    uint16_t size = RC_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(p_uart->hdmarx);

    dr16_rx_size = size;
    if (size == RC_FRAME_LENGTH) {
        DR16_ParseData(rx_buffer);
        if (DR16_DataIsValid()) {
            data_ready = 1;
            dr16_frame_count++;
        } else {
            dr16_drop_count++;
        }
    } else {
        dr16_drop_count++;
    }

    HAL_UART_DMAStop(p_uart);
    DR16_StartReceive();
}

uint8_t DR16_HasNewData(void)
{
    return data_ready;
}

void DR16_ClearDataFlag(void)
{
    data_ready = 0;
}
