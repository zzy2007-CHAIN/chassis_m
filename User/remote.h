#ifndef __remote_H
#define __remote_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define RC_FRAME_LENGTH     18
#define RC_CH_VALUE_OFFSET  1024
#define RC_CH_VALUE_MIN     364
#define RC_CH_VALUE_MAX     1684

// 拨杆状态值
#define RC_SW_UP    1
#define RC_SW_MID   3
#define RC_SW_DOWN  2

// 键盘按键位（key.v 的 bit 定义）
#define KEY_W       (1 << 0)
#define KEY_S       (1 << 1)
#define KEY_A       (1 << 2)
#define KEY_D       (1 << 3)
#define KEY_Q       (1 << 4)
#define KEY_E       (1 << 5)
#define KEY_SHIFT   (1 << 6)
#define KEY_CTRL    (1 << 7)

// 遥控器数据包结构（__packed 确保字节对齐）
typedef struct __attribute__((packed)) {
    struct {
        uint16_t ch0;       // 0~1684, 中位 1024
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t  s1;        // 拨杆状态 1/3/2
        uint8_t  s2;
    } rc;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct {
        uint16_t v;         // 键盘按键位
    } key;
} RC_Ctl_t;

void DR16_Init(UART_HandleTypeDef *huart);
RC_Ctl_t *DR16_GetData(void);
uint8_t DR16_HasNewData(void);
void DR16_ClearDataFlag(void);

extern volatile uint16_t dr16_rx_size;
extern volatile uint32_t dr16_frame_count;
extern volatile uint32_t dr16_drop_count;
extern volatile uint8_t dr16_raw_data[RC_FRAME_LENGTH];
extern volatile uint8_t dr16_valid;
extern volatile uint16_t dr16_ch0;
extern volatile uint16_t dr16_ch1;
extern volatile uint16_t dr16_ch2;
extern volatile uint16_t dr16_ch3;

// Called from ISR when USART idle detected
void DR16_IdleIRQHandler(void);

#endif
