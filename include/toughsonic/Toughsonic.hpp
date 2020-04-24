#ifndef TOUGHSONIC_HPP
#define TOUGHSONIC_HPP

#include <SerialStream.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <functional>

using namespace LibSerial;
class Toughsonic{
public:
    
    struct SensorConfig{
        std::string filename = "/dev/ttyUSB0";
        SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::BAUD_9600;
        SerialStreamBuf::CharSizeEnum characterSize = SerialStreamBuf::CHAR_SIZE_8;
        SerialStreamBuf::FlowControlEnum flowControl = SerialStreamBuf::FLOW_CONTROL_DEFAULT;
        SerialStreamBuf::ParityEnum parityType = SerialStreamBuf::PARITY_NONE;
        float stopBits = 1;
    };

    Toughsonic(SensorConfig sensorConfig);
    ~Toughsonic();

    void start(unsigned int updateIntervalMS);
    void close();

    void setSensorReadCallback(std::function<void(double)> callback);

private:    
    LibSerial::SerialStream sensorStream;
    
    std::atomic<bool> running{false};
    std::thread updateThread;

    std::function<void(double)> onSensorReadCallback;
};

#endif