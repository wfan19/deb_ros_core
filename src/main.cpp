#include "ros/ros.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "toughsonic");
    ros::NodeHandle n;

    ROS_INFO("Spinning node");
    while(ros::ok()){
        ros::spin();
    }

    // node has been terminated, end connection
    // <insert connection termination code>

    return 0;
}