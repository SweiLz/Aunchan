/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct Imu
{
	float gyro[3];  // X Y Z
	float accel[3]; // X Y Z
	float quat[4];  // W X Y Z
}Imu;

void setup(void);
void loop(void);
void imu2ros(const Imu _data);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
