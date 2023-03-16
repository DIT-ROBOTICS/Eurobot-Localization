#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

class P_control {

public :

    P_control(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

private :
	
    /* Function - for initialize params */
    void Initialize();

    /* Function - for update params */
    bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /* Function - for twist callback */
    void PoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /* Function publish sth we need */
    void publish();

    /** -- Node Handles -- **/
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;

    /** -- Advertise -- **/
    ros::Subscriber pose_sub_;
    ros::Subscriber quaternion_sub_;
	ros::Publisher vel_pub_;
    ros::ServiceServer param_srv_; // Service for update param ( call by other nodes )

    /** -- Params -- **/
    bool p_active_;
    bool p_publish_;
    bool p_update_params_;
    double p_goal_x_;
    double p_goal_y_;
    double p_goal_yaw_;
    double p_goal_round_;
    double p_max_vel_;
    double p_max_angular_vel_;
    double position_now_x = 0;
    double position_now_y = 0;
    double orientation_now_z_;
    double p_round_now_ = 0;

    bool p_with_p_control_;
    bool p_using_orientation_data_;

    std::string p_pub_topic_;
    std::string p_sub_topic_;

    /** -- Msgs to pub -- **/
    geometry_msgs::Twist vel_output_;

};