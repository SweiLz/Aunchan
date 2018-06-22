/*
 * main.cpp
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>

void leftPWMCallback(const std_msgs::Int16 & msg);
void rightPWMCallback(const std_msgs::Int16 & msg);
ros::NodeHandle nh;


char frame_id[] = "imu_frame";
sensor_msgs::Imu imu_msg;
std_msgs::Int64 l_enc_msg;
std_msgs::Int64 r_enc_msg;
std_msgs::Header header_msg;
ros::Publisher imu_pub("attitude", &imu_msg);
ros::Publisher l_enc_pub("l_enc", &l_enc_msg);
ros::Publisher r_enc_pub("r_enc", &r_enc_msg);

int16_t l_pwm = 500;
int16_t r_pwm = 500;
ros::Subscriber<std_msgs::Int16> l_pwm_sub("l_pwm", &leftPWMCallback);
ros::Subscriber<std_msgs::Int16> r_pwm_sub("r_pwm", &rightPWMCallback);
void leftPWMCallback(const std_msgs::Int16 & msg)
{
	l_pwm = msg.data;

}
void rightPWMCallback(const std_msgs::Int16 & msg)
{
	r_pwm = msg.data;
}
void ros2pwm(int16_t *pwm)
{
	pwm[0] = l_pwm;
	pwm[1] = r_pwm;
}
void encoder2ros(const int64_t *enc)
{
	l_enc_msg.data = enc[0];
	r_enc_msg.data = enc[1];
}
void imu2ros(const Imu _data)
{
	imu_msg.header.seq++;
	imu_msg.header.stamp = nh.now();
	imu_msg.orientation.w = _data.quat[0];
	imu_msg.orientation.x = _data.quat[1];
	imu_msg.orientation.y = _data.quat[2];
	imu_msg.orientation.z = _data.quat[3];
	imu_msg.angular_velocity.x = _data.gyro[0];
	imu_msg.angular_velocity.y = _data.gyro[1];
	imu_msg.angular_velocity.z = _data.gyro[2];
	imu_msg.linear_acceleration.x = _data.accel[0];
	imu_msg.linear_acceleration.y = _data.accel[1];
	imu_msg.linear_acceleration.z = _data.accel[2];

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(l_enc_pub);
  nh.advertise(r_enc_pub);
  nh.subscribe(l_pwm_sub);
  nh.subscribe(r_pwm_sub);
  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = frame_id;
}

void loop(void)
{
  imu_pub.publish(&imu_msg);
  l_enc_pub.publish(&l_enc_msg);
  r_enc_pub.publish(&r_enc_msg);
  nh.spinOnce();
}

void DebugROS (const char * Data)
{
	nh.loginfo(Data);
}
