#ifndef __DWT_H
#define __DWT_H

#include <stdint.h>

void DWT_Init(void);
void delayUS_DWT(uint32_t us);
void delay_us(uint32_t us);

#endif //__DWT_H
