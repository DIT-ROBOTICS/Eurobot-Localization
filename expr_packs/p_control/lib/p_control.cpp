#include "p_control/p_control.h"

P_control::P_control (ros::NodeHandle &nh, ros::NodeHandle &nh_local){

    this->nh_ = nh;
    this->nh_local_ = nh_local;
    this->Initialize();
}

void P_control::Initialize(){

    std_srvs::Empty empty;
	
    this->p_active_ = false;
    ROS_INFO_STREAM("[P CONTROL] : inactive node");

    if(this->UpdateParams(empty.request, empty.response)){
        ROS_INFO_STREAM("[P CONTROL] : Initialize param ok");
    }
    else {
        ROS_INFO_STREAM("[P CONTROL] : Initialize param failed");    
    } 

}

bool P_control::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    bool prev_active = p_active_;

    /* get param */
    if(this->nh_local_.param<bool>("active", p_active_, true)){
        ROS_INFO_STREAM("[P CONTROL] : active set to " << p_active_);
    }

    if(this->nh_local_.param<bool>("publish", p_publish_, true)){
        ROS_INFO_STREAM("[P CONTROL] : publish set to " << p_publish_);
    }

    if(this->nh_local_.param<std::string>("sub_topic", p_sub_topic_, "/global_filter")){
        ROS_INFO_STREAM("[P CONTROL] : Current subscribe topic [ " << p_sub_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("pub_topic", p_pub_topic_, "/cmd_vel")){
        ROS_INFO_STREAM("[P CONTROL] : Current publish topic [ " << p_pub_topic_ << " ]"); 
    }

    if(this->nh_local_.param<bool>("update_params", p_update_params_, false)){
        ROS_INFO_STREAM("[P CONTROL] : update params set to " << p_update_params_); 
    }

    if(this->nh_local_.param<double>("goal_x", p_goal_x_, 10.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_x set to " << p_goal_x_); 
    }

    if(this->nh_local_.param<double>("goal_y", p_goal_y_, 10.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_y set to " << p_goal_y_); 
    }

    if(this->nh_local_.param<double>("max_linear_vel", p_max_vel_, 0.3)){
        ROS_INFO_STREAM("[P CONTROL] : Max linear velocity set to " << p_max_vel_); 
    }

    if(this->nh_local_.param<double>("max_angular_vel", p_max_angular_vel_, 1.5)){
        ROS_INFO_STREAM("[P CONTROL] : Max angular velocity set to " << p_max_angular_vel_); 
    }

    if(this->nh_local_.param<bool>("with_p_control", p_with_p_control_, true)){
        ROS_INFO_STREAM("[P CONTROL] : With p_control set to " << p_with_p_control_); 
    }

    if(this->nh_local_.param<bool>("using orientation data", p_using_orientation_data_, false)){
        ROS_INFO_STREAM("[P CONTROL] : Using orientation data set to " << p_using_orientation_data_); 
    }

    if(this->nh_local_.param<double>("goal_yaw", p_goal_yaw_, 90.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_yaw set to " << p_goal_yaw_); 
    }

    if(this->nh_local_.param<double>("round", p_goal_round_, 1.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_round set to " << p_goal_round_); 
    }



    if(p_active_ != prev_active) {

        if (p_active_) {

            ROS_INFO_STREAM("[P CONTROL] : active node");
            this->pose_sub_ = nh_.subscribe(p_sub_topic_, 10, &P_control::PoseCallback, this);
            this->vel_pub_ = nh_.advertise<geometry_msgs::Twist>(p_pub_topic_, 10);

            if(this->p_update_params_){
                this->param_srv_ = nh_local_.advertiseService("params", &P_control::UpdateParams, this);
            }
        }
        else {
            this->pose_sub_.shutdown();
            this->vel_pub_.shutdown();

            if(this->p_update_params_){
                this->param_srv_.shutdown();
            }
        }
    }

    return true;
}    

void P_control::PoseCallback(const nav_msgs::Odometry::ConstPtr &msg){
    
    this->position_now_x = msg->pose.pose.position.x;
    this->position_now_y = msg->pose.pose.position.y;

    // linear velocity
    if( (this->p_goal_x_ - this->position_now_x) > 0.001 || (this->p_goal_y_ - this->position_now_y) > 0.001){
        
        if(this->p_with_p_control_){

            this->vel_output_.linear.x = (this->p_max_vel_ / 100.0) * (this->p_goal_x_ - this->position_now_x);
            this->vel_output_.linear.y = (this->p_max_vel_ / 100.0) * (this->p_goal_y_ - this->position_now_y);
        
        }

    }else{
        this->vel_output_.linear.x = 0.0;
        this->vel_output_.linear.y = 0.0;
    }

    // angular velocity

    if(this->p_using_orientation_data_){

        // 紀錄目前圈數

        tf::Quaternion q;
        // Set the quaternion values from the message
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

        this->orientation_now_z_ = tf::getYaw(q) * 180 / M_PI;

        // Convert quaternion to rotation matrix
        tf::Matrix3x3 m(q);

        Eigen::Matrix3d eigen_m;
        eigen_m << m[0][0], m[0][1], m[0][2],
                   m[1][0], m[1][1], m[1][2],
                   m[2][0], m[2][1], m[2][2];

        // Compute the eigenvalues and eigenvectors of the rotation matrix
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(eigen_m);
        Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();

        // Find the index of the maximum eigenvalue
        int max_index = 0;
        double max_value = eigenvalues(0);
        for (int i = 1; i < 3; ++i) {
            if (eigenvalues(i) > max_value) {
                max_index = i;Eigen/Denseation circles
        double angle = acos(max_value) * 2.0;
        this->p_round_now_ = angle / (2.0 * M_PI);

        // ROS_INFO("Number of rotation circles: %f", num_circles);

        if( (this->p_round_now_ < this->p_goal_round_) ){
        
            vel_output_.angular.z = this->p_max_angular_vel_;

        }else{

            if(this->p_with_p_control_){

                this->vel_output_.angular.z = (this->p_max_angular_vel_ / 100.0) * (this->p_goal_round_ - this->p_round_now_);
        
            }if(this->orientation_now_z_< 0 && this->orientation_now_z_ > -0.01){

                this->vel_output_.linear.x = 0.0;
                this->vel_output_.linear.y = 0.0;
            }
        }
    }

    if(this->p_publish_) this->publish();

}

void P_control::publish(){

    this->vel_pub_.publish(this->vel_output_);	

}