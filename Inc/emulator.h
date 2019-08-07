#ifndef __EMULATOR__H
#define __EMULATOR__H

#define EMU_STOP_EMULATION 0

enum STATUS {
    STATE_IDLE,
    STATE_ERROR,
    STATE_NEED_EMULATION,
    STATE_EMULATION,
    STATE_UNKNOWN,
};

enum STATUS_EMULATION {
    STATE_KEY_IDLE,
    STATE_KEY_1,
    STATE_KEY_2,
    STATE_KEY_3,
    STATE_KEY_4,
    STATE_KEY_5,
    STATE_KEY_6,
    STATE_KEY_7,
    STATE_KEY_8,
    STATE_KEY_9,
    STATE_KEY_10,
    STATE_KEY_FINISH,
    STATE_KEY_UNKNOWN,
};

enum STATUS_EMU_TIME {
    STATE_EMU_IDLE,
    STATE_EMU_PROCESS,
    STATE_EMU_UNKNOWN,
};

typedef struct Joystick Joystick;

typedef unsigned char uint8_t;

void emulator(volatile Joystick *rep, uint8_t frames);

#endif //__EMULATOR__H

