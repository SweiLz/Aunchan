/*
 * main.cpp
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

void messageCallback(const geometry_msgs::Twist&);
void messageCallback2(const std_msgs::Float64MultiArray&);

ros::NodeHandle nh;

//------IMU------//
char frame_id[] = "imu_frame";
sensor_msgs::Imu imu_msg;
std_msgs::Header header_msg;
ros::Publisher imu_pub("attitude", &imu_msg);

//------Odometry------//
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped odom_trans;
geometry_msgs::Quaternion odom_quat;
//std_msgs::Float64MultiArray all_msg;

Twist vel;

float pid[3] = {1.9, 17.8, 0.018};
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &messageCallback);
//ros::Subscriber<std_msgs::Float64MultiArray> pid_sub("pid", &messageCallback2);

void messageCallback(const geometry_msgs::Twist& msg)
{
	vel.linear[0] = msg.linear.x;
	vel.linear[1] = msg.linear.y;
	vel.linear[2] = msg.linear.z;
	vel.angular[0] = msg.angular.x;
	vel.angular[1] = msg.angular.y;
	vel.angular[2] = msg.angular.z;
}
void messageCallback2(const std_msgs::Float64MultiArray& msg)
{
	pid[0] = msg.data[0];
	pid[1] = msg.data[1];
	pid[2] = msg.data[2];
}
void ros2velocity(Twist *_vel)
{
	*_vel = vel;
}
void ros2pid(float *_pid)
{
	_pid[0] = pid[0];
	_pid[1] = pid[1];
	_pid[2] = pid[2];
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
void odometryUpdate(const Imu imu, const Twist vel)
{
  odom_quat.w = imu.quat[0];
  odom_quat.x = imu.quat[1];
  odom_quat.y = imu.quat[2];
  odom_quat.z = imu.quat[3];
  odom_msg.header.stamp = odom_trans.header.stamp = nh.now();
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;
  odom_trans.transform.rotation = odom_quat;
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = z;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.linear.x = vel.linear[0];
  odom_msg.twist.twist.linear.y = vel.linear[1];
  odom_msg.twist.twist.angular.z = imu.gyro[2];
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
  nh.subscribe(cmd_sub);
  //nh.subscribe(pid_sub);
  vel.linear[0] = 0;
  vel.linear[1] = 0;
  vel.linear[2] = 0;
  vel.angular[0] = 0;
  vel.angular[1] = 0;
  vel.angular[2] = 0;
  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = frame_id;

  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_msg.header.frame_id = "odom";
}

void loop(void)
{
  imu_pub.publish(&imu_msg);
  //------Odometry------//
  odom_broadcaster.sendTransform(odom_trans);
  odom_pub.publish(odom_msg);


  nh.spinOnce();
}

void DebugROS (const char * Data)
{
	nh.loginfo(Data);
}
