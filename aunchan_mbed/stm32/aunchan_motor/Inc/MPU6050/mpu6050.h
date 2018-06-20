/*
 * mpu6050.h
 *
 *  Created on: 11 Jun 2018
 *      Author: PRUEK
 */

#ifndef MPU6050_MPU6050_H_
#define MPU6050_MPU6050_H_

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stm32f1xx_hal.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

#define DEFAULT_MPU_HZ  (100)
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define ACCEL_SENS  16384.0
#define GYRO_SENS   131.0
#define QUAT_SENS   1073741824.0
#define PI 3.14159

struct hal_s {
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned short report;
	unsigned short dmp_features;
	unsigned char motion_int_mode;
};
void mpu6050_init();
void mpu6050_read(float *gyro, float *accel, float *quat);
int getms (uint32_t *count);
HAL_StatusTypeDef writeI2C (uint8_t DevAddress, uint8_t RegAddress, uint8_t len, uint8_t *pData);
HAL_StatusTypeDef readI2C (uint8_t DevAddress, uint8_t RegAddress, uint8_t len, uint8_t *pData);

#endif /* MPU6050_MPU6050_H_ */
