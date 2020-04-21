#ifndef TOUGHSONIC_HPP
#define TOUGHSONIC_HPP

#include "ros/ros.h"

#include <SerialStream.h>

class Toughsonic{
public:

    struct SensorConfig{
        float baudrate;
        std::string filename;
    };

    Toughsonic(ros::NodeHandle nh, SensorConfig sensorConfig);
    ~Toughsonic();

private:    
    ros::NodeHandle n;
    LibSerial::SerialStream sensorStream;


};

#endif