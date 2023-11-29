/*
 * lsm6dsl.c
 *
 *  Created on: Nov 9, 2023
 *      Author: wuyou
 */

#include "lsm6dsl.h"
#include "i2c.h"
#include "stdio.h"

#define LSM6DSL_ADDRESS 0b1101010 // Slave address. SA0 connected to ground by default

void lsm6dsl_init() {

	// 1. Write CTRL1_XL = 60h, Acc = 416 Hz (High-Performance mode)
	uint8_t ctrl1_xl_data[2] = {0x10, 0x60}; // CTRL1_XL register address, value
	i2c_transaction(LSM6DSL_ADDRESS, 0, ctrl1_xl_data, 2);

	// 2. Write INT1_CTRL = 01h, Acc data-ready interrupt on INT1
	uint8_t int1_ctrl_data[2] = {0x0D, 0x01};
	i2c_transaction(LSM6DSL_ADDRESS, 0, int1_ctrl_data, 2);

}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {

	/* Arrays initially filled with addresses of acceleration data registers,
	 * later used to store acceleration values
	 */
	uint8_t acceleration_x_l[1] = {0x28};
	uint8_t acceleration_x_h[1] = {0x29};
	uint8_t acceleration_y_l[1] = {0x2A};
	uint8_t acceleration_y_h[1] = {0x2B};
	uint8_t acceleration_z_l[1] = {0x2C};
	uint8_t acceleration_z_h[1] = {0x2D};

	// Write address of register for lower part of acceleration in x-direction
	i2c_transaction(LSM6DSL_ADDRESS, 0, acceleration_x_l, 1);
	// Read lower part of acceleration in x-direction
	i2c_transaction(LSM6DSL_ADDRESS, 1, acceleration_x_l, 1);
	// Write address of register for higher part of acceleration in x-direction
	i2c_transaction(LSM6DSL_ADDRESS, 0, acceleration_x_h, 1);
	// Read higher part of acceleration in x-direction
	i2c_transaction(LSM6DSL_ADDRESS, 1, acceleration_x_h, 1);

	/* Similar for acceleration in y-direction */
	i2c_transaction(LSM6DSL_ADDRESS, 0, acceleration_y_l, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, acceleration_y_l, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 0, acceleration_y_h, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, acceleration_y_h, 1);

	/* Similar for acceleration in z-direction */
	i2c_transaction(LSM6DSL_ADDRESS, 0, acceleration_z_l, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, acceleration_z_l, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 0, acceleration_z_h, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, acceleration_z_h, 1);

	/* Combine acceleration data and put them into pointers x, y, and z respectively */
	*x = (int16_t)((acceleration_x_h[0] << 8) | acceleration_x_l[0]);
	*y = (int16_t)((acceleration_y_h[0] << 8) | acceleration_y_l[0]);
	*z = (int16_t)((acceleration_z_h[0] << 8) | acceleration_z_l[0]);
}
