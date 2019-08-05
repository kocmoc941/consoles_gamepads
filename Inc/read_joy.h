#ifndef __READ_JOY__H
#define __READ_JOY__H

#include <stdint.h>

typedef enum {
    KEY_PRESS,
    KEY_RELEASE,
} KEY_STATUS;

typedef enum {
    KEY_A,
    KEY_B,
    KEY_SELECT,
    KEY_START,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
} Keys;

typedef enum {
    HID_ID,
    HID_X,
    HID_Y,
    HID_BUTTONS,
} HID_Keys;

typedef struct Joystick Joystick;

typedef struct {
    void (*init)(void);
    void (*read_joys)(void);
    void (*send_report)(void);
    uint8_t (*get_key)(Keys key);
    void (*set_key)(Keys key, KEY_STATUS stat);
} Joy_Control;

#define readTwoGamepads(byte1, byte2)    \
        NES_JOY1_LATCH(GPIO_PIN_SET);    \
        NES_JOY2_LATCH(GPIO_PIN_SET);    \
        delay_us(5);                     \
        NES_JOY1_LATCH(GPIO_PIN_RESET);  \
        NES_JOY2_LATCH(GPIO_PIN_RESET);  \
        delay_us(5);                     \
        byte1 = 0x00;                    \
        byte2 = 0x00;                    \
        for(int i = 0; i < 8; ++i) {     \
            {                            \
                NES_JOY1_CLK(GPIO_PIN_RESET);\
                NES_JOY2_CLK(GPIO_PIN_RESET);\
                delay_us(3);                 \
                const uint8_t bit = NES_JOY1_DATA; \
                byte1 |= ((bit & 0x1) << i);       \
                BLINK_LED(bit);                \
            }                                  \
            {                                  \
                const uint8_t bit = NES_JOY2_DATA; \
                byte2 |= ((bit & 0x1) << i);       \
                BLINK_LED(bit);                \
            }                                  \
            NES_JOY1_CLK(GPIO_PIN_SET);        \
            NES_JOY2_CLK(GPIO_PIN_SET);        \
            delay_us(5);                       \
        }

#define readTwoGamepadsAtSEGA(byte1, port1, sel, up_z, down_y, left_x \
                        , byte2, port2, right_mode, A_B, C_start)     \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);                  \
        delayUS_DWT(20);                                              \
                                                                      \
        byte1 = 0;                                                    \
        byte2 = 0;                                                    \
        const uint8_t up = HAL_GPIO_ReadPin(port1, up_z);             \
        const uint8_t down = HAL_GPIO_ReadPin(port1, down_y);         \
        const uint8_t left = HAL_GPIO_ReadPin(port1, left_x);         \
        const uint8_t right = HAL_GPIO_ReadPin(port2, right_mode);    \
        const uint8_t B = HAL_GPIO_ReadPin(port2, A_B);               \
        const uint8_t C = HAL_GPIO_ReadPin(port2, C_start);           \
        byte1 |= (up << 0);                                           \
        byte1 |= (down << 1);                                         \
        byte1 |= (left << 2);                                         \
        byte1 |= (right << 3);                                        \
        byte2 |= (C << 2);                                            \
        byte2 |= (B << 1);                                      \
                                                                \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t A = HAL_GPIO_ReadPin(port2, A_B);         \
        const uint8_t START = HAL_GPIO_ReadPin(port2, C_start); \
        byte2 |= (A << 0);/*A*/                                 \
        byte2 |= (START << 3);/*START*/                         \
                                                                \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t Z = HAL_GPIO_ReadPin(port1, up_z);        \
        const uint8_t Y = HAL_GPIO_ReadPin(port1, down_y);      \
        const uint8_t X = HAL_GPIO_ReadPin(port1, left_x);      \
        const uint8_t MODE = HAL_GPIO_ReadPin(port2, right_mode); \
        byte2 |= (X << 4); /*X*/                                \
        byte2 |= (Y << 5); /*Y*/                                \
        byte2 |= (Z << 6); /*Z*/                                \
        byte2 |= (MODE << 7); /*MODE*/                          \
                                                                \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        HAL_GPIO_WritePin(port1, sel, GPIO_PIN_SET);            \
        delayUS_DWT(20);

void InitControl_Joysticks(Joy_Control *joy);

#endif //__READ_JOY__H

