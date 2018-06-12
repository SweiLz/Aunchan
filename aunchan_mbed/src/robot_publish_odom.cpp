#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"


double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void cmd_velCB(const geometry_msgs::Twist& msg)
{
    vx = msg.linear.x;
    vy = msg.linear.y;
    vth = msg.angular.z;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_publish_odom");
    
    ros::NodeHandle n;
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel",50,&cmd_velCB);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10.0);
    
    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx*cos(th) - vy *sin(th)) * dt;
        double delta_y = (vx*sin(th) + vy *cos(th)) * dt;
        double delta_th = vth*dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}

