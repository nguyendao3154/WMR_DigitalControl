#include "systick.h"


void UserSysTick_Start() {
	HAL_TIM_Base_Start_IT(&htim1);
}

void UserSysTick_Stop() {
	HAL_TIM_Base_Stop_IT(&htim1);
}

uint32_t UserSysTick_GetTick() {
	return htim1.Instance->CNT;
}