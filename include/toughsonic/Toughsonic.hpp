#ifndef TOUGHSONIC_HPP
#define TOUGHSONIC_HPP

#include "ros/ros.h"

#include <SerialStream.h>
#include <atomic>
#include <thread>
#include <chrono>

using namespace LibSerial;
class Toughsonic{
public:
    
    struct SensorConfig{
        std::string filename = "/dev/ttyUSB0";
        SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::BaudRateEnum::BAUD_9600;
        SerialStreamBuf::CharSizeEnum characterSize = SerialStreamBuf::CharSizeEnum::CHAR_SIZE_8;
        SerialStreamBuf::FlowControlEnum flowControl = SerialStreamBuf::FlowControlEnum::FLOW_CONTROL_DEFAULT;
        SerialStreamBuf::ParityEnum parityType = SerialStreamBuf::ParityEnum::PARITY_NONE;
        float stopBits = 1;
    };

    Toughsonic(ros::NodeHandle nh, SensorConfig sensorConfig);
    ~Toughsonic();

    void start(unsigned int updateIntervalMS);
    void close();

private:    
    ros::NodeHandle n;
    LibSerial::SerialStream sensorStream;
    
    std::atomic<bool> running{false};
    std::thread updateThread;


};

#endif