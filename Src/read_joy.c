#include "config.h"
#include "dwt.h"
#include "read_joy.h"
#include "emulator.h"

#include <string.h>

#include "usbd_custom_hid_if.h"
#include "stm32f1xx_hal.h"

#define IS_PRESS(byte, btn) (!((byte >> btn) & 1))

uint8_t assert[(sizeof(struct Joystick) == 16) ? 1 : -1];

extern USBD_HandleTypeDef hUsbDeviceFS;

static void m_init(void);
static void m_read_joys(void);
static void m_send_report(void);
static uint8_t m_get_key(NES_Keys key);
static void m_set_key(NES_Keys key, KEY_STATUS stat);

volatile Joystick m_data;

inline void InitControl_Joysticks(Joy_Control *joy)
{
    joy->init = m_init;
    joy->read_joys = m_read_joys;
    joy->send_report = m_send_report;
    joy->get_key = m_get_key;
    joy->set_key = m_set_key;
}

inline static void m_init(void)
{
    memset((void *)&m_data, 0x0, sizeof(Joystick));
}

inline static void m_read_joys(void)
{
    uint8_t byte1 = 0xFF;
    uint8_t byte2 = 0xFF;
    readTwoGamepads(byte1, byte2)

    volatile Joystick *rep = &m_data;
    rep->n_joy1.buttons = (~byte1 & 0x0F);
    rep->n_joy2.buttons = (~byte2 & 0x0F);

    // Y
    if (IS_PRESS(byte1, NES_KEY_UP)) {
        rep->n_joy1.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, NES_KEY_DOWN)) {
        rep->n_joy1.up_down = (uint8_t)(127);
    } else {
        rep->n_joy1.up_down = 0x00;
    }

    // X
    if (IS_PRESS(byte1, NES_KEY_LEFT)) {
        rep->n_joy1.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, NES_KEY_RIGHT)) {
        rep->n_joy1.left_right = (uint8_t)(127);
    } else {
        rep->n_joy1.left_right = 0x00;
    }
    
    // Y
    if (IS_PRESS(byte2, NES_KEY_UP)) {
        rep->n_joy2.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte2, NES_KEY_DOWN)) {
        rep->n_joy2.up_down = (uint8_t)(127);
    } else {
        rep->n_joy2.up_down = 0x00;
    }

    // X
    if (IS_PRESS(byte2, NES_KEY_LEFT)) {
        rep->n_joy2.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte2, NES_KEY_RIGHT)) {
        rep->n_joy2.left_right = (uint8_t)(127);
    } else {
        rep->n_joy2.left_right = 0x00;
    }

    byte1 = 0xFF;
    byte2 = 0xFF;
    uint8_t byte3 = 0xFF;
    uint8_t byte4 = 0xFF;
    readTwoGamepadsAtSEGA(byte1, byte2, byte3, byte4)
    if (IS_PRESS(byte1, SEGA_KEY_LEFT)) {
        rep->s_joy1.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, SEGA_KEY_RIGHT)) {
        rep->s_joy1.left_right = (uint8_t)(127);
    } else {
        rep->s_joy1.left_right = 0x00;
    }

    if (IS_PRESS(byte1, SEGA_KEY_UP)) {
        rep->s_joy1.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, SEGA_KEY_DOWN)) {
        rep->s_joy1.up_down = (uint8_t)(127);
    } else {
        rep->s_joy1.up_down = 0x00;
    }

	rep->s_joy1.buttons = ~byte2;

    if (IS_PRESS(byte3, SEGA_KEY_LEFT)) {
        rep->s_joy2.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte3, SEGA_KEY_RIGHT)) {
        rep->s_joy2.left_right = (uint8_t)(127);
    } else {
        rep->s_joy2.left_right = 0x00;
    }

    if (IS_PRESS(byte3, SEGA_KEY_UP)) {
        rep->s_joy2.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte3, SEGA_KEY_RIGHT)) {
        rep->s_joy2.up_down = (uint8_t)(127);
    } else {
        rep->s_joy2.up_down = 0x00;
    }

    rep->s_joy2.buttons = ~byte4;

    rep->n_joy1.id = NES_JOY1;
    rep->n_joy2.id = NES_JOY2;
    rep->s_joy1.id = SEGA_JOY1;
    rep->s_joy2.id = SEGA_JOY2;
    
    //emulator(rep, 1);
}

inline static uint8_t m_get_key(NES_Keys key)
{
	return (uint8_t)((m_data.n_joy1.buttons >> key) & 0x1);
}

inline static void m_set_key(NES_Keys key, KEY_STATUS stat)
{
	m_data.n_joy1.buttons &= ~(uint8_t)(0x1 << key);
	m_data.n_joy1.buttons |= (uint8_t)(stat << key);
}

inline static void m_send_report(void)
{
    if(m_data.n_joy1.buttons || m_data.n_joy2.buttons || m_data.s_joy1.buttons || m_data.s_joy2.buttons
        || m_data.n_joy1.left_right || m_data.n_joy1.up_down
        || m_data.n_joy2.left_right || m_data.n_joy2.up_down
        || m_data.s_joy1.left_right || m_data.s_joy1.up_down
        || m_data.s_joy2.left_right || m_data.s_joy2.up_down) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    }
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&m_data, sizeof(m_data));
}
