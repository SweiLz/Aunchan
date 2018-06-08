/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
std_msgs::Float64 float_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher yeah("yeah", &float_msg);
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
  nh.advertise(yeah);
  nh.advertise(imu);
}

void loop(void)
{
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

  str_msg.data = hello;
  float_msg.data = 103.5;
  imu_msg.header.frame_id = "imu_frame";
  imu_msg.header.stamp = nh.now();
  chatter.publish(&str_msg);
  yeah.publish(&float_msg);
  imu.publish(&imu_msg);
  nh.spinOnce();

  HAL_Delay(20);
}

