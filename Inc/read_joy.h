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

#define readTwoGamepadsAtSEGA(byte1, byte2)											      \
        SEGA_JOY1_SELECT(GPIO_PIN_SET);                  \
        delayUS_DWT(20);                                              \
                                                                      \
        byte1 = 0x00;                                                    \
        byte2 = 0x00;                                                    \
        const uint8_t up = SEGA_JOY1_UP_Z;             \
        const uint8_t down = SEGA_JOY1_DOWN_Y;         \
        const uint8_t left = SEGA_JOY1_LEFT_X;         \
        const uint8_t right = SEGA_JOY1_RIGHT_MODE;    \
        const uint8_t B = SEGA_JOY1_A_B;               \
        const uint8_t C = SEGA_JOY1_C_START;           \
        byte1 |= (up << 0);                                           \
        byte1 |= (down << 1);                                         \
        byte1 |= (left << 2);                                         \
        byte1 |= (right << 3);                                        \
        byte2 |= (C << 2);                                            \
        byte2 |= (B << 1);                                      \
                                                                \
        SEGA_JOY1_SELECT(GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t A = SEGA_JOY1_A_B;         \
        const uint8_t START = SEGA_JOY1_C_START; \
        byte2 |= (A << 0);/*A*/                                 \
        byte2 |= (START << 3);/*START*/                         \
                                                                \
        SEGA_JOY1_SELECT(GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
        SEGA_JOY1_SELECT(GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        SEGA_JOY1_SELECT(GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
        SEGA_JOY1_SELECT(GPIO_PIN_RESET);           \
        delayUS_DWT(20);                                        \
        SEGA_JOY1_SELECT(GPIO_PIN_SET);            \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t Z = SEGA_JOY1_UP_Z;        \
        const uint8_t Y = SEGA_JOY1_DOWN_Y;      \
        const uint8_t X = SEGA_JOY1_LEFT_X;      \
        const uint8_t MODE = SEGA_JOY1_RIGHT_MODE; \
        byte2 |= (X << 4); /*X*/                                \
        byte2 |= (Y << 5); /*Y*/                                \
        byte2 |= (Z << 6); /*Z*/                                \
        byte2 |= (MODE << 7); /*MODE*/                          \
                                                                \
        SEGA_JOY1_SELECT(GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
        SEGA_JOY1_SELECT(GPIO_PIN_SET);            \
        delayUS_DWT(20);

void InitControl_Joysticks(Joy_Control *joy);

#endif //__READ_JOY__H

