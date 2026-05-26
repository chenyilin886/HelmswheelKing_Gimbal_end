#include "vofa.h"
#include "usart.h"

static uint8_t send_str2[sizeof(float) * 7];
static volatile uint8_t vofa_busy = 0;

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
{
    if (vofa_busy) return;
    vofa_busy = 1;

    const uint8_t sendSize = sizeof(float); 

    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; 
    
    HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(send_str2));
}

void vofa_tx_complete(void)
{
    vofa_busy = 0;
}