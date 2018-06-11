/*
 * main.cpp
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher imu("imu", &imu_msg);
char hello[] = "Hello world!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(imu);
}

void loop(void)
{

  str_msg.data = hello;
  imu_msg.header.frame_id = "imu_frame";
  imu_msg.header.stamp = nh.now();
  chatter.publish(&str_msg);
  imu.publish(&imu_msg);
  nh.spinOnce();

  HAL_Delay(50);
}

