#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

//std_msgs::Float64 yaw;
geometry_msgs::Point angle;

//float yaw = 0;

nav_msgs::Odometry odom1;
nav_msgs::Odometry odom2;
nav_msgs::Odometry odom3;

void call_back_1(const nav_msgs::Odometry::ConstPtr &msg1){

    odom1 = *msg1;
    tf::Quaternion q1(0, 0, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w);
    angle.x = tf::getYaw(q1) * 180 / M_PI;

}

void call_back_2(const nav_msgs::Odometry::ConstPtr &msg2){

    odom2 = *msg2;
    tf::Quaternion q2(0, 0, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w);
    angle.y = tf::getYaw(q2) * 180 / M_PI;
   
}

void call_back_3(const nav_msgs::Odometry::ConstPtr &msg3){

    odom3 = *msg3;
    tf::Quaternion q3(0, 0, odom3.pose.pose.orientation.z, odom3.pose.pose.orientation.w);
    angle.z = tf::getYaw(q3) * 180 / M_PI;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "getYaw");
    ros::NodeHandle nh;
    ros::Subscriber quaternion_sub1 = nh.subscribe("/robot1/test_odom", 10, call_back_1);
    ros::Subscriber quaternion_sub2 = nh.subscribe("/robot1/test_imu", 10, call_back_2);
    ros::Subscriber quaternion_sub3 = nh.subscribe("/robot1/test_all", 10, call_back_3);
    ros::Publisher local_filter_angle_pub = nh.advertise<geometry_msgs::Point>("orientation_angle",1);
    // float freq = 100;
    // ros::Rate rate(freq);
    float z1;
    int ss = 1;

    while(ros::ok()){

        ros::spinOnce();

        // if(angle.z >= (-180) && angle.z <= 180 && ss==1){
        //     z1 = angle.z;
        //     ss = 0;
        // }

        // std::cout << "X : " << angle.x << std::endl;
        // std::cout << "Y : " << angle.y << std::endl;
        // std::cout << "Z : " << angle.z << std::endl << "--------------------------" << std::endl;

        local_filter_angle_pub.publish(angle);
        // rate.sleep();
    }
    
}