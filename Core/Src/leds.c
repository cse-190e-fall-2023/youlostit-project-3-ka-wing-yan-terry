/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
  /* Enable the GPIOA clock */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  /* From schematic: LED 1 - PA5 (GPIO A Pin 5); LED2 - PB14 */
  /* Configure PA5 as an output by clearing all bits and setting the mode */
  GPIOA->MODER &= ~GPIO_MODER_MODE5; // GPIO_MODER_MODE5 has all bits set to 0 except those corresponding to pin 5 set to 1
  // Set bits corresponding to PA5 to 0 while other bits retain
  GPIOA->MODER |= GPIO_MODER_MODE5_0; // Set 01 to the position of PA5

  GPIOB->MODER &= ~GPIO_MODER_MODE14;
  GPIOB->MODER |= GPIO_MODER_MODE14_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

  GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

  GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

  /* Configure the GPIO pin to use low speed mode */
  GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

  /* Turn off the LED */
  GPIOA->ODR &= ~GPIO_ODR_OD5;

  GPIOB->ODR &= ~GPIO_ODR_OD14;
}

void leds_set(uint8_t led)
{

	if(led == 0b01) {
		GPIOA->ODR |= GPIO_ODR_OD5;
		GPIOB->ODR &= ~GPIO_ODR_OD14;
	}
	else if(led == 0b10) {
		GPIOA->ODR &= ~GPIO_ODR_OD5;
		GPIOB->ODR |= GPIO_ODR_OD14;
	}
	else if(led == 0b11){
		GPIOA->ODR |= GPIO_ODR_OD5;
		GPIOB->ODR |= GPIO_ODR_OD14;
	}
	else {
		GPIOA->ODR &= ~GPIO_ODR_OD5;
		GPIOB->ODR &= ~GPIO_ODR_OD14;
	}
}
