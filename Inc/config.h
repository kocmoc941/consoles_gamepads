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

#define SEGA_JOY1_UP_Z 		    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)
#define SEGA_JOY1_DOWN_Y      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)
#define SEGA_JOY1_LEFT_X      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)
#define SEGA_JOY1_RIGHT_MODE  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)
#define SEGA_JOY1_A_B         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)
#define SEGA_JOY1_C_START     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)
#define SEGA_JOY1_SELECT(val) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, val)
#define SEGA_JOY2_UP_Z 		    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)
#define SEGA_JOY2_DOWN_Y      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)
#define SEGA_JOY2_LEFT_X      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)
#define SEGA_JOY2_RIGHT_MODE  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#define SEGA_JOY2_A_B         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)
#define SEGA_JOY2_C_START     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define SEGA_JOY2_SELECT(val) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, val)

#define BLINK_LED(val) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, val ? GPIO_PIN_SET : GPIO_PIN_RESET)

#endif //__CONFIG_H
