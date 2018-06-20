/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

typedef struct Twist
{
	float linear[3]; // X Y Z
	float angular[3]; // X Y Z
}Twist;

typedef struct Imu
{
	float gyro[3];  // X Y Z
	float accel[3]; // X Y Z
	float quat[4];  // W X Y Z
}Imu;

#ifdef __cplusplus
 extern "C" {
#endif

void setup(void);
void loop(void);
void imu2ros(const Imu);
void ros2velocity(Twist*);
void ros2pid(float*);
void DebugROS (const char *);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
