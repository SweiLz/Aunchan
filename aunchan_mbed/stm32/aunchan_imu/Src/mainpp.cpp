/*
 * main.cpp
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;


sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("attitude", &imu_msg);
char frame_id[] = "imu_frame";
std_msgs::Header header_msg;

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

  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = frame_id;
}

void loop(void)
{
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
}

