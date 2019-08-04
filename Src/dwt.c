#include "main.h"

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

void DWT_Init(void)
{
     SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
     DWT_CYCCNT  = 0;
     DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
}

void delayUS_DWT(uint32_t us) {
    volatile uint32_t cycles = (SystemCoreClock/1000000L) * us;
    volatile uint32_t start = DWT->CYCCNT;
    do  {
    } while((DWT->CYCCNT - start) < cycles);
}

void delay_us(uint32_t us)
{
    DWT_Init();
    delayUS_DWT(us);
}
