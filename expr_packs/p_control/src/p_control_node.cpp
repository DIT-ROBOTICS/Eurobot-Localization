#include "p_control/p_control.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "test_control_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    try {

        ROS_INFO_STREAM("[P_CONTROL] : Initializing p_control node");
        P_control p_control(nh, nh_local);
        ros::spin();
    }
    catch (const char* s) {
    	
        ROS_FATAL_STREAM("[P_CONTROL] : " << s);
  	
    }
    catch (...)	{
    	
        ROS_FATAL_STREAM("[P_CONTROL] : Unexpected error");

    }

     return 0;
}
