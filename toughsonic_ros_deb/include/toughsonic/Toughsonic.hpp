#ifndef TOUGHSONIC_HPP
#define TOUGHSONIC_HPP

#include <SerialStream.h>
#include <iostream>
#include <string>
#include <vector>

using namespace LibSerial;
class Toughsonic
{
public:
    struct SensorConfig
    {
        std::string filename = "/dev/ttyUSB0";
        SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::BAUD_9600;
        SerialStreamBuf::CharSizeEnum characterSize = SerialStreamBuf::CHAR_SIZE_8;
        SerialStreamBuf::FlowControlEnum flowControl = SerialStreamBuf::FLOW_CONTROL_DEFAULT;
        SerialStreamBuf::ParityEnum parityType = SerialStreamBuf::PARITY_NONE;
        float stopBits = 1;
    };

    Toughsonic(SensorConfig sensorConfig);
    ~Toughsonic();

    double read();

private:    
    LibSerial::SerialStream sensorStream;
};

#endif