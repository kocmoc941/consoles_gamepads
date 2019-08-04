#ifndef __READ_JOY__H
#define __READ_JOY__H

#include <stdint.h>

#pragma pack(push, 1)
typedef struct {
    struct NES {
        uint8_t id;
        uint8_t up:1;
        uint8_t down:1;
        uint8_t left:1;
        uint8_t right:1;
        uint8_t select:1;
        uint8_t start:1;
        uint8_t a:1;
        uint8_t b:1;
    } n_joy1, n_joy2;
        struct SEGA {
        uint8_t id;
        uint8_t up:1;
        uint8_t down:1;
        uint8_t left:1;
        uint8_t right:1;
        uint8_t b:1;
        uint8_t c:1;
        uint8_t a:1;
        uint8_t start:1;
        uint8_t z:1;
        uint8_t y:1;
        uint8_t x:1;
        uint8_t mode:1;
    } s_joy1, s_joy2;
} Joystick;
#pragma pack(pop, 1)

typedef struct {
    void (*init)(Joystick *joy);
    void (*read_joys)(void);
    void (*send_report)(void);

    uint8_t __data1;
    uint8_t __data2;
    uint8_t __data3;
    uint8_t __data4;
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

#endif //__READ_JOY__H

