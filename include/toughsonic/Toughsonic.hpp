#ifndef TOUGHSONIC_HPP
#define TOUGHSONIC_HPP

#include "ros/ros.h"

#include <SerialStream.h>
#include <atomic>
#include <thread>
#include <chrono>

class Toughsonic{
public:
    struct SensorConfig{
        std::string filename = "/dev/ttyUSB0";
        LibSerial::SerialStreamBuf::BaudRateEnum baudRate = LibSerial::SerialStreamBuf::BaudRateEnum::BAUD_9600;
        LibSerial::SerialStreamBuf::CharSizeEnum characterSize = LibSerial::SerialStreamBuf::CharSizeEnum::CHAR_SIZE_8;
        LibSerial::SerialStreamBuf::FlowControlEnum flowControl = LibSerial::SerialStreamBuf::FlowControlEnum::FLOW_CONTROL_DEFAULT;
        LibSerial::SerialStreamBuf::ParityEnum parityType = LibSerial::SerialStreamBuf::ParityEnum::PARITY_NONE;
        float stopBits = 1;
    };

    Toughsonic(ros::NodeHandle nh, SensorConfig sensorConfig);
    ~Toughsonic();

    void start(unsigned int updateIntervalMS);

private:    
    ros::NodeHandle n;
    LibSerial::SerialStream sensorStream;
    atomic<bool> running;


};

#endif