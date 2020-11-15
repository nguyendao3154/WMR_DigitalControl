#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include "tim.h"

void UserSysTick_Start(void);
uint32_t UserSysTick_GetTick(void);
void UserSysTick_Stop(void);

#endif
