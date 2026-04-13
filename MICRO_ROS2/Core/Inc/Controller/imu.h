#ifndef IMU_H
#define IMU_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>

// Expose the calculated angles for use in your micro-ROS publisher
extern float Ax;
extern float Ay;
extern float Gz;

// Function Prototypes
bool MPU6050_Init(I2C_HandleTypeDef *hi2c);
bool MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c);
bool MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c);
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c);

#endif // IMU_H