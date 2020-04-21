#include "ros/ros.h"
#include "../include/toughsonic/Toughsonic.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "toughsonic");
    ros::NodeHandle n;


    Toughsonic::SensorConfig sensorConfig;
    sensorConfig.baudrate = 9600;
    sensorConfig.filename = "/dev/ttyUSB0";
    Toughsonic mToughsonic(n, sensorConfig);

    ROS_INFO("Spinning node");
    while(ros::ok()){
        ros::spin();
    }

    // node has been terminated, end connection
    // <insert connection termination code>

    return 0;
}