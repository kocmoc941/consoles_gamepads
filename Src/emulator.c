#include "emulator.h"
#include "read_joy.h"

#include <string.h>

#define IS_PRESS(byte, btn) ((byte >> btn) & 1)

enum STATUS m_status = STATE_IDLE;
enum STATUS_EMULATION m_status_emu = STATE_KEY_IDLE;
enum STATUS_EMU_TIME m_status_emu_time = STATE_EMU_IDLE;

uint8_t m_frames = 0;
volatile uint8_t m_tmp[3] = {0x0};

void emulator(volatile Joystick *rep, uint8_t frames)
{
    const uint8_t C = IS_PRESS(rep->s_joy1.buttons, SEGA_KEY_C);
    const uint8_t Z = IS_PRESS(rep->s_joy1.buttons, SEGA_KEY_Z);
    if( !C && !Z) {
        m_tmp[0] = 0x00;
        m_tmp[1] = 0x00;
        m_tmp[2] = 0x00;
        m_status_emu = STATE_KEY_IDLE;
        return;
    }

    volatile uint8_t *LR = &m_tmp[0];
    volatile uint8_t *UD = &rep->s_joy1.up_down;
    volatile uint8_t *BT = &m_tmp[2];
		//*BT |= rep->s_joy1.buttons;

    switch(m_status) {
        case STATE_IDLE: {
            if(frames == EMU_STOP_EMULATION) {
                m_status_emu = STATE_KEY_FINISH;
            } else if (m_status_emu == STATE_KEY_IDLE) {
                m_status = STATE_NEED_EMULATION;
            }
            break;
        }

        case STATE_NEED_EMULATION: {
            m_status = STATE_EMULATION;
            m_status_emu = (enum STATUS_EMULATION)(m_status_emu + STATE_KEY_1);
            m_status_emu_time = STATE_EMU_PROCESS; 
            ++m_frames;
            break;
        }
        case STATE_EMULATION: {
            ++m_frames;
            break;
        }
        case STATE_ERROR: {
            break;
        }
        case STATE_UNKNOWN: {
            break;
        }

    };

    switch(m_status_emu) {
        case STATE_KEY_IDLE: {
            break;
        }
        case STATE_KEY_1: {
            if(Z) {
                *LR = (uint8_t)-127;
            } else {
                *LR = (uint8_t)127;
            }
            break;
        }
        case STATE_KEY_2: {
            *LR = 0;
            break;
        }
        case STATE_KEY_3: {
            if(Z) {
                *LR = (uint8_t)-127;
            } else {
                *LR = (uint8_t)127;
            }
            break;
        }
        case STATE_KEY_4: {
            *BT = 2;
            break;
        }
        case STATE_KEY_5: {
            break;
        }
        case STATE_KEY_6: {
            break;
        }
        case STATE_KEY_7: {
            break;
        }
        case STATE_KEY_8: {
            break;
        }
        case STATE_KEY_9: {
            break;
        }
        case STATE_KEY_10: {
            break;
        }
        case STATE_KEY_FINISH: {
            m_frames = 0;
            m_status = STATE_IDLE;
            //m_status_emu = STATE_KEY_IDLE;
            m_status_emu_time = STATE_EMU_IDLE;
            break;
        }
        case STATE_KEY_UNKNOWN: {
            break;
        }
    };

    switch(m_status_emu_time) {
        case STATE_EMU_IDLE: {
            break;
        }
        case STATE_EMU_PROCESS: {
            if(m_frames == frames) {
                m_status = STATE_NEED_EMULATION;
                m_frames = 0;
            }
            break;
        }
        case STATE_EMU_UNKNOWN: {
            break;
        }
    };

    rep->s_joy1.left_right = *LR;
    rep->s_joy1.up_down = *UD;
    rep->s_joy1.buttons = *BT;
}

