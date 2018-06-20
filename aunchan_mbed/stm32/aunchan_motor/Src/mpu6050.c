/*
 * mpu6050.c
 *
 *  Created on: 11 Jun 2018
 *      Author: PRUEK
 */

#include"mpu6050.h"

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static inline int run_self_test(void);

void mpu6050_init ()
{
  static struct hal_s hal = {0};
  static signed char gyro_orientation[9] = { 0,  -1, 0,
                                             1,   0, 0,
                                             0,   0, 1};
  int result;
  int result_selftest;
  struct int_param_s int_param;
  unsigned char  accel_fsr;
  unsigned short gyro_rate, gyro_fsr;

  if(HAL_I2C_IsDeviceReady(&hi2c1,(uint16_t)0xD0,2,HAL_MAX_DELAY) != HAL_OK){
	 //printf("Device not ready\r\n");
  }

  uint8_t data = 0x80;

  if (writeI2C((uint8_t)0xD0,(uint8_t)0x6B,(uint8_t)1, &data)){
	 //printf("ERROR2\r\n");
  }

  HAL_Delay(100);

  result = mpu_init(&int_param);
  if (result) {
	 //printf("Could not initialize gyro.\r\n");
  }

  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  mpu_set_gyro_fsr(250);
  mpu_set_accel_fsr(2);
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  memset(&hal,0,sizeof(hal));
  dmp_load_motion_driver_firmware();
  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
  hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(hal.dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  mpu_set_dmp_state(1);
  hal.dmp_on = 1;

  //printf("Sample Rate = %hd\r\n",gyro_rate);
  //printf("Gyro Full Scale Range = %u\r\n",gyro_fsr);
  //printf("Accel Full Scale Range = %u\r\n",accel_fsr);
  result_selftest = run_self_test();

  if (result_selftest == 0x7){
	  //printf("-----Test Pass-----\r\n");
  }

  HAL_Delay(1000);
}

void mpu6050_read(float *gyro, float *accel, float *quat)
{
	short _gyro[3]  = {};
	short _accel[3] = {};
	long  _quat[4]  = {};
	unsigned long sensor_timestamp;
	short sensors;
	unsigned char more;

	dmp_read_fifo(_gyro, _accel, _quat, &sensor_timestamp, &sensors, &more);

	quat[0] = (float)_quat[0] / QUAT_SENS;
	quat[1] = (float)_quat[1] / QUAT_SENS;
	quat[2] = (float)_quat[2] / QUAT_SENS;
	quat[3] = (float)_quat[3] / QUAT_SENS;
	gyro[0] = (float)_gyro[0] / GYRO_SENS;
	gyro[1] = (float)_gyro[1] / GYRO_SENS;
	gyro[2] = (float)_gyro[2] / GYRO_SENS;
	accel[0] = (float)_accel[0] / ACCEL_SENS;
	accel[1] = (float)_accel[1] / ACCEL_SENS;
	accel[2] = (float)_accel[2] / ACCEL_SENS;
	/*yaw   = atan2(2.0f * (quatf[1] * quatf[2] + quatf[0] * quatf[3]), quatf[0] * quatf[0] + quatf[1] * quatf[1] - quatf[2] * quatf[2] - quatf[3] * quatf[3]);
	pitch = -asin(2.0f * (quatf[1] * quatf[3] - quatf[0] * quatf[2]));
	roll  = atan2(2.0f * (quatf[0] * quatf[1] + quatf[2] * quatf[3]), quatf[0] * quatf[0] - quatf[1] * quatf[1] - quatf[2] * quatf[2] + quatf[3] * quatf[3]);
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI;
	roll  *= 180.0f / PI;*/
}
HAL_StatusTypeDef readI2C (uint8_t DevAddress, uint8_t RegAddress, uint8_t len, uint8_t *pData)
{
	HAL_StatusTypeDef returnValue;

	returnValue = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &RegAddress, 1, HAL_MAX_DELAY);
	if(returnValue != HAL_OK)
			return returnValue;

	returnValue = HAL_I2C_Master_Receive(&hi2c1, DevAddress, pData, len, HAL_MAX_DELAY);
	return returnValue;
}

HAL_StatusTypeDef writeI2C (uint8_t DevAddress, uint8_t RegAddress, uint8_t len, uint8_t *pData)
{
	HAL_StatusTypeDef returnValue;
	uint8_t *data;

	data = (uint8_t*)malloc(sizeof(uint8_t) * (len + 1));
	data[0] = RegAddress;

	memcpy(data + 1, pData, len);

	returnValue = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, data, len + 1, HAL_MAX_DELAY);
	if(returnValue != HAL_OK)
		return returnValue;

	free(data);

	//while(HAL_I2C_Master_Transmit(hi2c, DevAddress, 0, 0, HAL_MAX_DELAY) != HAL_OK);

	//return HAL_OK;
	return returnValue;
}

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

int getms (uint32_t *count){
	*count = HAL_GetTick();
	return 0;
}

static inline int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    unsigned char i = 0;

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

        mpu_set_accel_bias_6050_reg(accel);
        return result;
    }
    return result;
}
