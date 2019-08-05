#ifndef __CONFIG_H
#define __CONFIG_H

#define _DEBUG

// joys id
#define NES_JOY1  0x1
#define NES_JOY2  0x2
#define SEGA_JOY1 0x3
#define SEGA_JOY2 0x4

//configuration pins
#define NES_JOY1_LATCH(val) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, val)
#define NES_JOY1_CLK(val)   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, val)
#define NES_JOY1_DATA       HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#define NES_JOY2_LATCH(val) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, val)
#define NES_JOY2_CLK(val)   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, val)
#define NES_JOY2_DATA       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)

#define BLINK_LED(val) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, val ? GPIO_PIN_SET : GPIO_PIN_RESET)

#endif //__CONFIG_H
