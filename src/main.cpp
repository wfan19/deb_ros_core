#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <toughsonic/Toughsonic.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "toughsonic");
    ros::NodeHandle n;

    ros::Publisher distance_pub = n.advertise<std_msgs::Float64>("toughsonic/distance/value", 1000);

    Toughsonic::SensorConfig sensor_config;
    sensor_config.filename = "/dev/ttyUSB0";
    sensor_config.baudRate = LibSerial::SerialStreamBuf::BAUD_9600;
    Toughsonic mToughsonic(sensor_config);

    ros::Rate sensor_frequency(10);

    ROS_INFO("Spinning node");
    while(ros::ok()){
        ros::spinOnce();
        
        double distance = mToughsonic.read();
        std_msgs::Float64 distance_msg;
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);

        sensor_frequency.sleep();
    }

    return 0;
}