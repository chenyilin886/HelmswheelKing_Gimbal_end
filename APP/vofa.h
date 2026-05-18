#ifndef VOFA_H
#define VOFA_H
#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
void vofa_tx_complete(void);

#ifdef __cplusplus
}
#endif

#endif 
