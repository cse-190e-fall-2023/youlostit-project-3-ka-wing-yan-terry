/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{

	if(timer == TIM2) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Enable TIM2 clock
	}

	if(timer == TIM3) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; // Enable TIM3 clock
	}

	timer->CR1 &= ~TIM_CR1_CEN; // Stop timer
	timer->SR = 0; // Clear timer status register
	timer->CNT = 0; // Clear timer count register

	timer->ARR = 0xFFFFFFFF; // Set auto-reload value to maximum (32-bit timer, upcounting)

	timer->DIER |= TIM_DIER_UIE; // Enable timer update interrupt internally

	if(timer == TIM2) {
		NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 update interrupt NVIC
		NVIC_SetPriority(TIM2_IRQn, 0); // Set priority of the interrupt
	}

	if(timer == TIM3) {
		NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 update interrupt NVIC
		NVIC_SetPriority(TIM3_IRQn, 0); // Set priority of the interrupt
	}

	timer->PSC = 200;// Decrease timer frequency to 4000000 / 200 = 20000 Hz

	timer->CR1 |= TIM_CR1_CEN; // Enable timer

}

void timer_reset(TIM_TypeDef* timer)
{
	timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	timer->ARR = 20 * period_ms - 1; // In each cycle, timer counts {ARR} times to reach {peroid_ms} ms
}
