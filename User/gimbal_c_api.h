#ifndef GIMBAL_C_API_H
#define GIMBAL_C_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void Gimbal_Init(void);
    void Gimbal_InitCANBus(void);
    void Gimbal_StartUARTReceive(void);
    void Gimbal_StartIMUReceive(void);
    void Gimbal_EnableMotors(void);
    void Gimbal_DisableMotors(void);
    void Gimbal_Update(void);
    void Gimbal_SendToChassis(void);
    void Gimbal_ProcessUARTRx(void *huart, uint16_t size);
    void Gimbal_ProcessUARTRxCplt(void *huart);
    void Gimbal_ProcessCANRx(void *hcan);
    void Gimbal_ParseCANFrame(void *frame);
    void Gimbal_ProcessCANFifo0(void *hcan);
    void Gimbal_ProcessCANFifo1(void *hcan);
    
    uint8_t* Gimbal_GetVisionRxBuffer(void);
    uint8_t Gimbal_GetVisionRxSize(void);

#ifdef __cplusplus
}
#endif

#endif
