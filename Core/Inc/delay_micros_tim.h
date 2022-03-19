/*
 * delay_micros_tim.h
 *
 *  Created on: Jan 4, 2022
 *      Author: dima
 */

#ifndef INC_DELAY_MICROS_TIM_H_
#define INC_DELAY_MICROS_TIM_H_

#include "main.h"

#define TIMUS            TIM7

__STATIC_INLINE void Delay_us_tim_init()
{
	TIMUS->ARR = (uint32_t)65535;
	TIMUS->PSC = (64 - 1);
	TIMUS->EGR = TIM_EGR_UG;
	TIMUS->SR = ~TIM_SR_UIF;
	TIMUS->CR1 |= TIM_CR1_CEN;   // запускаем timer
}

__STATIC_INLINE void delay_us(uint16_t us)
{
	TIMUS->CNT = 0;
	while(TIMUS->CNT < us);
}


#endif /* INC_DELAY_MICROS_TIM_H_ */
