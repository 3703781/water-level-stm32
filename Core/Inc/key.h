#ifndef _KEY_H
#define _KEY_H
#include "stm32f1xx_hal.h"

#define KEY0_PRES 1
#define KEY1_PRES 2
#define WKUP_PRES 3

void KEY_Init(void);
uint8_t KEY_Scan(uint8_t mode);
#endif
