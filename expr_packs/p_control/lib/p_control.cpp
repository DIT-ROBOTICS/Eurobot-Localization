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

    if(this->nh_local_.param<std::string>("sub_topic_2", p_sub_topic_2_, "/lightgate")){
        ROS_INFO_STREAM("[P CONTROL] : Current subscribe topic [ " << p_sub_topic_2_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("pub_topic", p_pub_topic_, "/cmd_vel")){
        ROS_INFO_STREAM("[P CONTROL] : Current publish topic [ " << p_pub_topic_ << " ]"); 
    }

    if(this->nh_local_.param<bool>("update_params", p_update_params_, false)){
        ROS_INFO_STREAM("[P CONTROL] : update params set to " << p_update_params_); 
    }

    if(this->nh_local_.param<double>("goal_x", p_goal_x_, 100.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_x set to " << p_goal_x_); 
    }

    if(this->nh_local_.param<double>("goal_y", p_goal_y_, 100.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_y set to " << p_goal_y_); 
    }

    if(this->nh_local_.param<double>("max_linear_vel", p_max_vel_, 0.3)){
        ROS_INFO_STREAM("[P CONTROL] : Max linear velocity set to " << p_max_vel_); 
    }

    if(this->nh_local_.param<double>("max_angular_vel", p_max_angular_vel_, 0.5)){
        ROS_INFO_STREAM("[P CONTROL] : Max angular velocity set to " << p_max_angular_vel_); 
    }

    if(this->nh_local_.param<double>("min_angular_vel", p_min_angular_vel_, 0.2)){
        ROS_INFO_STREAM("[P CONTROL] : Min angular velocity set to " << p_max_angular_vel_); 
    }

    if(this->nh_local_.param<double>("angular_accel", p_angular_accel_, 0.01)){
        ROS_INFO_STREAM("[P CONTROL] : Angular accel set to " << p_angular_accel_); 
    }

    if(this->nh_local_.param<bool>("with_p_control", p_with_p_control_, true)){
        ROS_INFO_STREAM("[P CONTROL] : With p_control set to " << p_with_p_control_); 
    }

    // this->p_linear_accel_x_ = this->p_max_vel_ / (this->p_goal_x_/4.0);

    if(this->nh_local_.param<bool>("using_linear_data", p_using_linear_data_, false)){
        ROS_INFO_STREAM("[P CONTROL] : Using linear data set to " << p_using_linear_data_); 
    }

    if(this->nh_local_.param<bool>("using_orientation_data", p_using_orientation_data_, false)){
        ROS_INFO_STREAM("[P CONTROL] : Using orientation data set to " << p_using_orientation_data_); 
    }

    if(this->nh_local_.param<double>("goal_yaw", p_goal_yaw_, 90.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_yaw set to " << p_goal_yaw_); 
    }

    if(this->nh_local_.param<double>("goal_round", p_goal_round_, 1.0)){
        ROS_INFO_STREAM("[P CONTROL] : Goal_round set to " << p_goal_round_); 
    }



    if(p_active_ != prev_active) {

        if (p_active_) {

            ROS_INFO_STREAM("[P CONTROL] : active node");
            this->pose_sub_ = nh_.subscribe(p_sub_topic_, 10, &P_control::PoseCallback, this);
            this->lightgate_sub_ = nh_.subscribe<std_msgs::Bool>(p_sub_topic_2_, 10, &P_control::LightGateCallback, this);
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
    if(this->p_using_linear_data_){
        if( (this->p_goal_x_ - fabs(this->position_now_x)) > 0.001 || (this->p_goal_y_ - (this->position_now_y)) > 0.001){
        
        if(this->p_with_p_control_){

            if((this->p_goal_x_-fabs(this->position_now_x)) > (this->p_goal_x_/2.0) && this->vel_output_.linear.x <= this->p_max_vel_){
                this->vel_output_.linear.x += 0.001;
                // std::cout << this->vel_output_.linear.x << std::endl;
            }
            if((this->p_goal_y_-fabs(this->position_now_y)) > (this->p_goal_y_/2.0) && this->vel_output_.linear.y <= this->p_max_vel_){
                this->vel_output_.linear.y += 0.001;
                // std::cout << this->vel_output_.linear.y << std::endl;
            }
    
            if((this->p_goal_x_-fabs(this->position_now_x)) < (this->p_goal_x_/2.0) && this->vel_output_.linear.x >= 0.05){

                // this->vel_output_.linear.x = (this->p_max_vel_) * (this->p_goal_x_ - this->position_now_x);
                // this->vel_output_.linear.y = (this->p_max_vel_) * (this->p_goal_y_ - this->position_now_y);
                this->vel_output_.linear.x -= 0.001; 
                // std::cout << this->vel_output_.linear.x << std::endl;
            }
            if((this->p_goal_x_-fabs(this->position_now_x)) < (this->p_goal_x_/2.0) && this->vel_output_.linear.x >= 0.05){

                this->vel_output_.linear.x -= 0.001; 
                // std::cout << this->vel_output_.linear.x << std::endl;
            }
        }

        }else{

            this->vel_output_.linear.x = 0.0;
            this->vel_output_.linear.y = 0.0;
        }
    }
    
    
    // angular velocity

    if(this->p_using_orientation_data_){

        // 紀錄目前圈數

        tf::Quaternion q;
        // Set the quaternion values from the message
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        this->orientation_now_z_ = tf::getYaw(q) * 180 / M_PI;

        // if(if_first){
        //     if(this->orientation_now_z_ > 0){
        //         p_or_n_last = 1;
        //         p_or_n = 1;
        //     }else{
        //         p_or_n_last = 0;
        //         p_or_n = 0;
        //     }
        //     if_first = 0;
        //     this->p_round_now_ = -0.5;
        // }else{
        //     p_or_n_last = p_or_n;
        // }

        // if(this->orientation_now_z_ > 0){
        //     p_or_n = 1;
        // }else{
        //     p_or_n = 0;
        // }

        // if(p_or_n != p_or_n_last){
        //     this->p_round_now_ += 0.5;
        // }  
        
        p_or_n_last = p_or_n;

        p_or_n = this->if_trigger_;

        if(p_or_n - p_or_n_last == 1){
            this->p_round_now_ += 1.0;
        } 

        std::cout << this->orientation_now_z_ << std::endl;
        std::cout << this->if_trigger_ << std::endl;
        std::cout << this->p_round_now_ << std::endl << "-----" << std::endl;

        if(this->p_round_now_ < 1.0){
            if(vel_output_.angular.z < this->p_max_angular_vel_){
                this->vel_output_.angular.z += 0.01;
            }
        }

    //     if(this->p_round_now_ == this->p_goal_round_-0.5 || this->p_round_now_ == this->p_goal_round_){

    //         if(fabs(this->orientation_now_z_) <= 1){

    //             for(int i=0;i<30;i++){
    //                 this->vel_output_.angular.z = 0.0;
    //             }

    //         }else{

    //             if(fabs(this->vel_output_.angular.z) < this->p_min_angular_vel_){

    //                 if(this->vel_output_.angular.z > 0.0){
    //                     this->vel_output_.angular.z = this->p_min_angular_vel_;
    //                 }else{
    //                     this->vel_output_.angular.z = -this->p_min_angular_vel_;
    //                 }
    //             }else if(this->vel_output_.angular.z > 0.0){
    //                 this->vel_output_.angular.z -= this->p_angular_accel_;
    //             }else{
    //                 this->vel_output_.angular.z += this->p_angular_accel_;
    //             }

    //         }
    //     }

    // }

    if(this->p_round_now_ == this->p_goal_round_- 1.0 || this->p_round_now_ == this->p_goal_round_){

        if(this->p_round_now_ == this->p_goal_round_ && this->if_trigger_ == 1){

            for(int i=0;i<30;i++){
                this->vel_output_.angular.z = 0.0;
            }

        }
        else{

            if(fabs(this->vel_output_.angular.z) < this->p_min_angular_vel_){

                if(this->vel_output_.angular.z > 0.0){
                    this->vel_output_.angular.z = this->p_min_angular_vel_;
                }else{
                    this->vel_output_.angular.z = -this->p_min_angular_vel_;
                }
            }else if(this->vel_output_.angular.z > 0.0){
                this->vel_output_.angular.z -= this->p_angular_accel_;
            }else{
                this->vel_output_.angular.z += this->p_angular_accel_;
            }
            }

        }
    

                
    }

    if(this->p_publish_) this->publish();

}


void P_control::publish(){

    this->vel_pub_.publish(this->vel_output_);	

}

void P_control::LightGateCallback(const std_msgs::Bool::ConstPtr &msg1){

    this->if_trigger_ = msg1->data;
    // std::cout << this->if_trigger_ << std::endl;

}