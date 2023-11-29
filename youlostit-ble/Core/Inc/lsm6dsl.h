#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <stdint.h>

// Function to initialize the LSM6DSL accelerometer
void lsm6dsl_init();

// Function to read the X, Y, and Z acceleration data from the LSM6DSL
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);

#endif
