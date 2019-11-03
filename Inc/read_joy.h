#ifndef __READ_JOY__H
#define __READ_JOY__H

typedef enum {
    KEY_PRESS,
    KEY_RELEASE,
} KEY_STATUS;

typedef enum {
    NES_KEY_A,
    NES_KEY_B,
    NES_KEY_SELECT,
    NES_KEY_START,
    NES_KEY_UP,
    NES_KEY_DOWN,
    NES_KEY_LEFT,
    NES_KEY_RIGHT,
} NES_Keys;

typedef enum {
    SEGA_KEY_UP,
    SEGA_KEY_DOWN,
    SEGA_KEY_LEFT,
    SEGA_KEY_RIGHT,
    SEGA_KEY_A = 0x0,
    SEGA_KEY_B,
    SEGA_KEY_C,
    SEGA_KEY_START,
    SEGA_KEY_X,
    SEGA_KEY_Y,
    SEGA_KEY_Z,
    SEGA_KEY_MODE,
} SEGA_Keys;

typedef enum {
    HID_ID,
    HID_X,
    HID_Y,
    HID_BUTTONS,
} HID_Keys;

typedef unsigned char uint8_t;

#pragma pack(push, 1)
struct Joystick {

    struct NES {
        uint8_t id;
        uint8_t left_right;
        uint8_t up_down;
        uint8_t buttons;
    } n_joy1, n_joy2;

    struct SEGA {
        uint8_t id;
        uint8_t left_right;
        uint8_t up_down;
        uint8_t buttons;
    } s_joy1, s_joy2;
};
#pragma pack(pop, 1)

typedef struct Joystick Joystick;

typedef struct {
    void (*init)(void);
    void (*read_joys)(void);
    void (*send_report)(void);
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

#define readTwoGamepadsAtSEGA(byte1, byte2, byte3, byte4)											      \
        SEGA_JOY1_SELECT(GPIO_PIN_SET);                  \
        delayUS_DWT(20);                                              \
                                                                      \
        byte1 = 0x00;                                                    \
        byte2 = 0x00;                                                    \
        byte3 = 0x00;                                                    \
        byte4 = 0x00;                                                    \
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
        byte2 |= (C << 1);                                            \
        byte2 |= (B << 2);                                      \
                                                                \
        SEGA_JOY1_SELECT(GPIO_PIN_RESET);          \
        delayUS_DWT(20);                                        \
                                                                \
        const uint8_t A = SEGA_JOY1_A_B;         \
        const uint8_t START = SEGA_JOY1_C_START; \
        byte2 |= (A << 3);/*A*/                                 \
        byte2 |= (START << 0);/*START*/                         \
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
        delayUS_DWT(20); \
        { \
                SEGA_JOY2_SELECT(GPIO_PIN_SET);                  \
                const uint8_t up = SEGA_JOY2_UP_Z;             \
                const uint8_t down = SEGA_JOY2_DOWN_Y;         \
                const uint8_t left = SEGA_JOY2_LEFT_X;         \
                const uint8_t right = SEGA_JOY2_RIGHT_MODE;    \
                const uint8_t B = SEGA_JOY2_A_B;               \
                const uint8_t C = SEGA_JOY2_C_START;           \
                byte3 |= (up << 0);                                           \
                byte3 |= (down << 1);                                         \
                byte3 |= (left << 2);                                         \
                byte3 |= (right << 3);                                        \
                byte4 |= (C << 1);                                            \
                byte4 |= (B << 2);                                      \
                                                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_RESET);          \
                delayUS_DWT(20);                                        \
                                                                        \
                const uint8_t A = SEGA_JOY2_A_B;         \
                const uint8_t START = SEGA_JOY2_C_START; \
                byte4 |= (A << 3);/*A*/                                 \
                byte4 |= (START << 0);/*START*/                         \
                                                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_SET);            \
                delayUS_DWT(20);                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_RESET);          \
                delayUS_DWT(20);                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_SET);            \
                delayUS_DWT(20);                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_RESET);           \
                delayUS_DWT(20);                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_SET);            \
                delayUS_DWT(20);                                        \
                                                                        \
                const uint8_t Z = SEGA_JOY2_UP_Z;        \
                const uint8_t Y = SEGA_JOY2_DOWN_Y;      \
                const uint8_t X = SEGA_JOY2_LEFT_X;      \
                const uint8_t MODE = SEGA_JOY2_RIGHT_MODE; \
                byte4 |= (X << 4); /*X*/                                \
                byte4 |= (Y << 5); /*Y*/                                \
                byte4 |= (Z << 6); /*Z*/                                \
                byte4 |= (MODE << 7); /*MODE*/                          \
                                                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_RESET);          \
                delayUS_DWT(20);                                        \
                SEGA_JOY2_SELECT(GPIO_PIN_SET);            \
                delayUS_DWT(20);    \
        }
        
void InitControl_Joysticks(Joy_Control *joy);

#endif //__READ_JOY__H
