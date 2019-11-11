#include "emulator.h"
#include "read_joy.h"

#include <string.h>
#include <stdint.h>

#define IS_PRESS(byte, btn) ((byte >> btn) & 1)

#define EMU_BUTTONS_MAX 30

enum BUTTON_NUM {
    BTN_FIRST,
    BTN_SECOND,
    BTN_THIRD,
    BTN_CNT,
};

typedef struct EMU_BUTTON {
    struct BUTTON {
        union {
            SEGA_Keys sega;
            NES_Keys nes;
        } key;
        KEY_Status status;
        uint32_t frame_cnt;
        uint8_t btn_active;
    } btn[BTN_CNT];
} EMU_Btn;

EMU_Btn m_emu_btns[EMU_BUTTONS_MAX];

uint32_t m_frames = 0;
uint32_t m_btn_cnt = 0;
uint32_t m_btn_emu = 0;

enum STATUS m_status = STATE_EMU_IDLE;

static void add_combo_init_once()
{
    static uint8_t one = 0;
    if(!one) {
        one = 1;
    } else {
        return;
    }

    #define add_key ++m_btn_cnt

    #define add_nes_key(__key_num, __key, __status, __frame_cnt) \
        m_emu_btns[m_btn_cnt].btn[##__key_num].btn_active = (1<<__key_num); \
        m_emu_btns[m_btn_cnt].btn[##__key_num].frame_cnt = __frame_cnt; \
        m_emu_btns[m_btn_cnt].btn[##__key_num].status = __status; \
        m_emu_btns[m_btn_cnt].btn[##__key_num].key.nes = (NES_Keys)(__key)

    add_nes_key(BTN_FIRST, NES_KEY_RIGHT, KEY_PRESS, 20);
    add_key;
    add_nes_key(BTN_FIRST, NES_KEY_RIGHT, KEY_RELEASE, 20);
    add_key;
    add_nes_key(BTN_FIRST, NES_KEY_RIGHT, KEY_PRESS, 10);
    //add_nes_key(BTN_SECOND, NES_KEY_B, KEY_PRESS, 10);
    add_key;
    add_nes_key(BTN_FIRST, NES_KEY_RIGHT, KEY_RELEASE, 10);
    //add_nes_key(BTN_SECOND, NES_KEY_B, KEY_RELEASE, 10);
    add_key;
}

void emulator(volatile Joystick *rep)
{
    add_combo_init_once();
    switch(m_status) {
        case STATE_EMU_IDLE: {
            if(m_btn_cnt) {
                ++m_frames;
                m_status = STATE_EMU_START_EMULATION;
            }
            break;
        }

        case STATE_EMU_START_EMULATION: {
            ++m_frames;
            m_status = STATE_EMU_EMULATION;
            break;
        }
        case STATE_EMU_EMULATION: {
            ++m_frames;
            if(m_emu_btns[m_btn_emu].btn[BTN_THIRD].btn_active == (1<<BTN_THIRD)) {
                if(m_emu_btns[m_btn_emu].btn[BTN_THIRD].status == KEY_PRESS)
                    rep->n_joy1.buttons = (1<<m_emu_btns[m_btn_emu].btn[BTN_THIRD].key.nes);
                else
                    rep->n_joy1.buttons = ~(1<<m_emu_btns[m_btn_emu].btn[BTN_THIRD].key.nes);
            }
            if(m_emu_btns[m_btn_emu].btn[BTN_THIRD].btn_active == (1<<BTN_SECOND)) {
                if(m_emu_btns[m_btn_emu].btn[BTN_SECOND].status == KEY_PRESS)
                    rep->n_joy1.buttons = (1<<m_emu_btns[m_btn_emu].btn[BTN_SECOND].key.nes);
                else
                    rep->n_joy1.buttons = ~(1<<m_emu_btns[m_btn_emu].btn[BTN_SECOND].key.nes);
            }
            if(m_emu_btns[m_btn_emu].btn[BTN_THIRD].btn_active == (1<<BTN_FIRST)) {
                if(m_emu_btns[m_btn_emu].btn[BTN_FIRST].status == KEY_PRESS)
                    rep->n_joy1.buttons = (1<<m_emu_btns[m_btn_emu].btn[BTN_FIRST].key.nes);
                else
                    rep->n_joy1.buttons = ~(1<<m_emu_btns[m_btn_emu].btn[BTN_FIRST].key.nes);
            }

            if(m_frames >= m_emu_btns[m_btn_emu].btn[BTN_THIRD].frame_cnt) {
                m_frames = 0;
                m_status = STATE_EMU_FINISH;
            }
            break;
        }
        case STATE_EMU_FINISH: {
            if(m_btn_emu < m_btn_cnt) {
                ++m_btn_emu;
                m_status = STATE_EMU_START_EMULATION;
            } else {
                m_btn_emu = 0;
                //m_status = STATE_EMU_IDLE;
            }
            break;
        }
        case STATE_EMU_ERROR: {
            break;
        }
        case STATE_EMU_UNKNOWN: {
            break;
        }

    };
}

