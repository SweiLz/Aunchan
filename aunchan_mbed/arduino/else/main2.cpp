#include <Arduino.h>
#include <Aunchan.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>

#define DEBUG_PERIOD 1               //hz
#define IMU_PUBLISH_PERIOD 200       //hz
#define INFOMATION_PUBLISH_PERIOD 30 //hz
#define CONTROL_PERIOD 15            //hz

#define WHEEL_RADIUS 0.035    //meter
#define WHEEL_SEPARATION 0.23 //meter
#define ROBOT_RADIUS 0.3      //meter

#define ENCODER_PER_REVOLUTE 1560 //raw

#define TICK2RAD 0.004027683 // 2*pi/encoder_per_revolute => 2*3.14159/1560

#define VELOCITY_CONSTANT_VALUE 7093.7632 // encoder_per_revolute/(2*pi)/wheel_radius => 1560/(2*3.14159)/0.035

Aunchan robot;
int16_t count = 0;
bool state = HIGH;
static uint32_t tTime[4]; //Software timer
char log_msg[255];

double goal_linear_velocity = 0;
double goal_angular_velocity = 0.08726646259971647;

float wheel_speed_cmd[2];
int16_t vel_sp[2];
int32_t pos_sp[2]={0,0};

float odom_pose[3] = {0.0, 0.0, 0.0};

unsigned long prev_update_time;

void controlMotorSpeed(void);
void publishImuMsg(void);
void publishBatteryMsg(void);
void blinkCb(const std_msgs::Empty &msg);
void cmdvelCb(const geometry_msgs::Twist &msg);

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu", &imu_msg);
std_msgs::Float32 battery_msg;
ros::Publisher battery_pub("/battery", &battery_msg);
nav_msgs::Odometry odom;
ros::Publisher odom_pub("/odom", &odom);
ros::Subscriber<std_msgs::Empty> blink_sub("/toggle_led", &blinkCb);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmdvelCb);

void M1_ISR()
{
    robot.motor[0]->intEnc();
}

void M2_ISR()
{
    robot.motor[1]->intEnc();
}

void M1_ISR2()
{
    robot.motor[0]->int2Enc();
}

void M2_ISR2()
{
    robot.motor[1]->int2Enc();
}

void setup()
{
    nh.initNode();
    nh.subscribe(blink_sub);
    nh.subscribe(cmd_vel_sub);
    nh.advertise(imu_pub);
    nh.advertise(battery_pub);
    tfbroadcaster.init(nh);
    nh.loginfo("Connected to Aunchan board!");
    attachInterrupt(M1_ENC_A, M1_ISR, CHANGE);
    attachInterrupt(M1_ENC_B, M1_ISR2, CHANGE);
    attachInterrupt(M2_ENC_A, M2_ISR, CHANGE);
    attachInterrupt(M2_ENC_B, M2_ISR2, CHANGE);
    prev_update_time = millis();
    delay(2000);
}

void loop()
{
    if ((millis() - tTime[0]) >= (1000 / DEBUG_PERIOD))
    {
        // sprintf(log_msg, "=> %d => %d", count++,a);
        // nh.loginfo(log_msg);
        tTime[0] = millis();
    }
    if ((millis() - tTime[1]) >= (1000 / IMU_PUBLISH_PERIOD))
    {
        // publishImuMsg();
        tTime[1] = millis();
    }
    if ((millis() - tTime[2]) >= (1000 / INFOMATION_PUBLISH_PERIOD))
    {
        // publishBatteryMsg();
        // publishOdomMsg();
        tTime[2] = millis();
    }
    if ((millis() - tTime[3]) >= (1000 / CONTROL_PERIOD))
    {
        controlMotorSpeed();
        tTime[3] = millis();
    }
    nh.spinOnce();
}

void controlMotorSpeed(void)
{

    unsigned long step_time = millis() - prev_update_time;

    wheel_speed_cmd[0] = goal_linear_velocity - (goal_angular_velocity * WHEEL_SEPARATION / 2);
    wheel_speed_cmd[1] = -(goal_linear_velocity + (goal_angular_velocity * WHEEL_SEPARATION / 2));

    vel_sp[0] = round((wheel_speed_cmd[0] * VELOCITY_CONSTANT_VALUE) * step_time * 0.001);
    vel_sp[1] = round((wheel_speed_cmd[1] * VELOCITY_CONSTANT_VALUE) * step_time * 0.001);
    pos_sp[0] += vel_sp[0];
    pos_sp[1] += vel_sp[1];
    // sprintf(log_msg,"Time => Steptime %f sec.",step_time*0.001);
    // sprintf(log_msg, "Tick => %ld/%d %ld/%d ", pos_sp[0],vel_sp[0], pos_sp[1], vel_sp[1]);
    // sprintf(log_msg,"Encoder => L: %ld R: %ld",pos_sp[0]-robot.motor[0]->getCount(),pos_sp[1]-robot.motor[1]->getCount());
    // sprintf(log_msg, "EncoderRev => L: %f R: %f", robot.motor[0]->Diff() * TICK2RAD, robot.motor[1]->Diff() * TICK2RAD);
    // if(abs(pos_sp[0]-robot.motor[0]->getCount())>5*abs(vel_sp[0]))
    // {
    //     robot.motor[0]->resetCount();
    //     pos_sp[0] = robot.motor[0]->getCount()+2*vel_sp[0];
    //     // robot.pid[0]->sum_error = 0;
    // }
    // if(abs(pos_sp[1]-robot.motor[1]->getCount())>5*abs(vel_sp[1]))
    // {
    //     robot.motor[1]->resetCount();
    //     pos_sp[1] = robot.motor[1]->getCount()+2*vel_sp[1];
    //     // robot.pid[1]->sum_error = 0;
    // }
    sprintf(log_msg, "Diff setpoint => L: %d/%d R: %d/%d", vel_sp[0],pos_sp[0]-robot.motor[0]->getCount(),vel_sp[1],pos_sp[1]-robot.motor[1]->getCount());
    // float m1_radsec = robot.motor[0]->Diff()*TICK2RAD/WHEEL_RADIUS;// / (step_time * 0.001);
    // float m2_radsec = robot.motor[1]->Diff()*TICK2RAD/WHEEL_RADIUS;// / (step_time * 0.001);
    // sprintf(log_msg, "Diff setpoint => L: %f \tR: %f", m1_radsec ,m2_radsec);
    nh.loginfo(log_msg);

    robot.speedControl(pos_sp[0], pos_sp[1]);

    prev_update_time = millis();
}
void publishImuMsg(void)
{
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = 1.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    imu_pub.publish(&imu_msg);

    tfs_msg.header.stamp = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id = "imu_link";
    tfs_msg.transform.rotation.w = 1.0;
    tfs_msg.transform.rotation.x = 0.0;
    tfs_msg.transform.rotation.y = 0.0;
    tfs_msg.transform.rotation.z = 0.0;

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.07;

    tfbroadcaster.sendTransform(tfs_msg);
}

void publishBatteryMsg(void)
{
    battery_msg.data = robot.getBattery();
    battery_pub.publish(&battery_msg);
}

void blinkCb(const std_msgs::Empty &msg)
{
    robot.setLED(state);
    state = !state;
}
void cmdvelCb(const geometry_msgs::Twist &msg)
{
    goal_linear_velocity = msg.linear.x;
    goal_angular_velocity = msg.angular.z;
}
