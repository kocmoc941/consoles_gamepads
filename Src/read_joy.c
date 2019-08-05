#include "config.h"
#include "dwt.h"
#include "read_joy.h"

#include <string.h>

#include "usbd_custom_hid_if.h"
#include "stm32f1xx_hal.h"

#define IS_PRESS(byte, btn) (!((byte >> btn) & 1))

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

uint8_t assert[(sizeof(struct Joystick) == 16) ? 1 : -1];

extern USBD_HandleTypeDef hUsbDeviceFS;

static void m_init(void);
static void m_read_joys(void);
static void m_send_report(void);
static uint8_t m_get_key(Keys key);
static void m_set_key(Keys key, KEY_STATUS stat);

volatile Joystick m_data;

void InitControl_Joysticks(Joy_Control *joy)
{
    joy->init = m_init;
    joy->read_joys = m_read_joys;
    joy->send_report = m_send_report;
    joy->get_key = m_get_key;
    joy->set_key = m_set_key;
}

static void m_init(void)
{
    memset((void *)&m_data, 0x0, sizeof(Joystick));
}

static void m_read_joys(void)
{
    uint8_t byte1 = 0xFF;
    uint8_t byte2 = 0xFF;
    readTwoGamepads(byte1, byte2)

    volatile Joystick *rep = &m_data;
    rep->n_joy1.buttons = (~byte1 & 0x0F);
    rep->n_joy2.buttons = (~byte2 & 0x0F);

    // Y
    if (IS_PRESS(byte1, KEY_UP)) {
        rep->n_joy1.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, KEY_DOWN)) {
        rep->n_joy1.up_down = (uint8_t)(127);
    } else {
        rep->n_joy1.up_down = 0x00;
    }

    // X
    if (IS_PRESS(byte1, KEY_LEFT)) {
        rep->n_joy1.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, KEY_RIGHT)) {
        rep->n_joy1.left_right = (uint8_t)(127);
    } else {
        rep->n_joy1.left_right = 0x00;
    }
    
    // Y
    if (IS_PRESS(byte2, KEY_UP)) {
        rep->n_joy2.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte2, KEY_DOWN)) {
        rep->n_joy2.up_down = (uint8_t)(127);
    } else {
        rep->n_joy2.up_down = 0x00;
    }

    // X
    if (IS_PRESS(byte2, KEY_LEFT)) {
        rep->n_joy2.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte2, KEY_RIGHT)) {
        rep->n_joy2.left_right = (uint8_t)(127);
    } else {
        rep->n_joy2.left_right = 0x00;
    }

    byte1 = 0xFF;
    byte2 = 0xFF;
    uint8_t byte3 = 0xFF;
    uint8_t byte4 = 0xFF;
    readTwoGamepadsAtSEGA(byte1, byte2, byte3, byte4)
    if (IS_PRESS(byte1, 2)) {
        rep->s_joy1.left_right = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, 3)) {
        rep->s_joy1.left_right = (uint8_t)(127);
    } else {
        rep->s_joy1.left_right = 0x00;
    }
		
    if (IS_PRESS(byte1, 0)) {
        rep->s_joy1.up_down = (uint8_t)(-127);
    } else if (IS_PRESS(byte1, 1)) {
        rep->s_joy1.up_down = (uint8_t)(127);
    } else {
        rep->s_joy1.up_down = 0x00;
    }
		
	rep->s_joy1.buttons = ~byte2;
    
    {
            if (IS_PRESS(byte3, 2)) {
                rep->s_joy2.left_right = (uint8_t)(-127);
            } else if (IS_PRESS(byte3, 3)) {
                rep->s_joy2.left_right = (uint8_t)(127);
            } else {
                rep->s_joy2.left_right = 0x00;
            }
                
            if (IS_PRESS(byte3, 0)) {
                rep->s_joy2.up_down = (uint8_t)(-127);
            } else if (IS_PRESS(byte3, 1)) {
                rep->s_joy2.up_down = (uint8_t)(127);
            } else {
                rep->s_joy2.up_down = 0x00;
            }
                
            rep->s_joy2.buttons = ~byte4;
    }
		
    rep->n_joy1.id = NES_JOY1;
    rep->n_joy2.id = NES_JOY2;
    rep->s_joy1.id = SEGA_JOY1;
    rep->s_joy2.id = SEGA_JOY2;
}

static uint8_t m_get_key(Keys key)
{
	return (uint8_t)((m_data.n_joy1.buttons >> key) & 0x1);
}

static void m_set_key(Keys key, KEY_STATUS stat)
{
	m_data.n_joy1.buttons &= ~(uint8_t)(0x1 << key);
	m_data.n_joy1.buttons |= (uint8_t)(stat << key);
}

static void m_send_report(void)
{
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&m_data, sizeof(m_data));
}
