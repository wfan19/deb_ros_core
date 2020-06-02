#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "../include/toughsonic/Toughsonic.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "toughsonic");
    ros::NodeHandle n;

    ros::Publisher distancePub = n.advertise<std_msgs::Float64>("toughsonic/distance/value", 1000);

    Toughsonic::SensorConfig sensorConfig;
    sensorConfig.filename = "/dev/ttyUSB0";
    sensorConfig.baudRate = LibSerial::SerialStreamBuf::BAUD_9600;
    Toughsonic mToughsonic(sensorConfig);
    mToughsonic.setSensorReadCallback([&](double dist){
        // V Logging message V
        ROS_INFO("Distance: %d", dist);
        
        std_msgs::Float64 distanceMsg;
        distanceMsg.data = dist;
        distancePub.publish(distanceMsg);
    });
    mToughsonic.start(1);

    ROS_INFO("Spinning node");
    while(ros::ok()){
        ros::spin();
    }

    mToughsonic.close();

    return 0;
}