#ifndef __DWT_H
#define __DWT_H

#include <stdint.h>

inline void DWT_Init(void);
inline void delayUS_DWT(uint32_t us);
inline void delay_us(uint32_t us);

#endif //__DWT_H
