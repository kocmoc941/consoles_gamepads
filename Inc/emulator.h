#ifndef __EMULATOR__H
#define __EMULATOR__H

#define EMU_STOP_EMULATION 0

enum STATUS {
    STATE_EMU_IDLE,
    STATE_EMU_START_EMULATION,
    STATE_EMU_EMULATION,
    STATE_EMU_FINISH,
    STATE_EMU_ERROR,
    STATE_EMU_UNKNOWN,
};

typedef struct Joystick Joystick;

inline void emulator(volatile Joystick *rep);

#endif //__EMULATOR__H

