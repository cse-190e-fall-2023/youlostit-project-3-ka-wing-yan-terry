/*
 * i2c.c
 *
 *  Created on: Nov 2, 2023
 *      Author: wuyou
 */

#include <stm32l475xx.h>

#include "stdio.h"

void i2c_init()
{
    // Enable I2C2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    // Enable clock for GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    // Clear PE bit in I2C_CR1
    I2C2->CR1 &= !I2C_CR1_PE;

    /*
     * Configure I2C2 SDA pin and I2C2 SCL pin.
     * Specifically, configure pins PB10 and PB11.
     */
    // Set PB10 and PB11 to alternate function mode
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk)) | (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);
    // Set PB10 and PB11 to output open-drain
    GPIOB->OTYPER |= (GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);
    // PB10-AF4, PB11-AF4
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos | 4 << GPIO_AFRH_AFSEL11_Pos);
    // Pull-up
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_0 |GPIO_PUPDR_PUPD11_0 );

    /*
     * I2C Timings
     * Configure PRESC[3:0],
     * SDADEL[3:0], SCLDEL[3:0], SCLH[7:0],
     * SCLL[7:0] in I2C_TIMINGR
     * Configure I2C2 to 100 kHz
     * Note: I2C APB clock (PCLK) default value 4 MHz, t_PCLK = 250 ns
     *
     */

    // Configure frequency of I2CCLK to 400 kHz
    RCC->CCIPR |= RCC_CCIPR_I2C2SEL_1; // Select HSI16 (16 MHz) as I2C2 clock, t_I2CCLK = 62.5 ns
    RCC->CR |= RCC_CR_HSION; // Enable HSI16
    I2C2->TIMINGR |= 1<<I2C_TIMINGR_PRESC_Pos;
    I2C2->TIMINGR |= 0x9<<I2C_TIMINGR_SCLL_Pos;
    I2C2->TIMINGR |= 0x3<<I2C_TIMINGR_SCLH_Pos;
    I2C2->TIMINGR |= 0x2<<I2C_TIMINGR_SDADEL_Pos;
    I2C2->TIMINGR |= 0x3<<I2C_TIMINGR_SCLDEL_Pos;

    // Configure NOSTRETCH in I2C_CR1
    I2C2->CR1 &= !I2C_CR1_NOSTRETCH;

    // Set PE bit in I2C_CR1
    I2C2->CR1 |= I2C_CR1_PE;

}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {

    while (I2C2->ISR & I2C_ISR_BUSY) { } // Wait until I2C2 bus is free

	/* Master communication initialization */
    // Set 7-bit slave address, R/W bit to write, NBYTES, and START bit.
    I2C2->CR2 = (address << 1) | (0 << I2C_CR2_RD_WRN_Pos) | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;

    if (dir == 0) { // Write

    	// Write register address and other data to I2C bus
        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_TXIS)) { } // Wait for TXIS = 1
            I2C2->TXDR = data[i];
        }

    }
    if (dir == 1) { // Read

    	// Wait for TXIS = 1
    	while (!(I2C2->ISR & I2C_ISR_TXIS)) { }
    	// Write register address to I2C bus
    	I2C2->TXDR = data[0];
    	// Wait for ACK
    	while(!(I2C2->ISR & I2C_ISR_TC));
    	// Set 7-bit slave address, R/W bit to read, NBYTES, and START bit.
        I2C2->CR2 = (address << 1) | (1 << I2C_CR2_RD_WRN_Pos) | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;

    	// Read data from I2C bus
        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_RXNE)) { } // Wait for RXNE = 1
            data[i] = I2C2->RXDR;
        }

    }

    // Wait for TC = 1
    while(!(I2C2->ISR & I2C_ISR_TC));

    if(I2C2->ISR & I2C_ISR_TC) {
		I2C2->CR2 |= I2C_CR2_STOP;// Generate stop condition
		return 1; // Success
    }
    else {
    	return 0;
    }
}
