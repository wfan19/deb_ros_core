#include "../include/toughsonic/Toughsonic.hpp"

Toughsonic::Toughsonic(ros::NodeHandle nh, Toughsonic::SensorConfig sensorConfig){
    this->n = nh;
    sensorStream.Open(sensorConfig.filename);
    sensorStream.SetBaudRate(sensorConfig.baudRate);
    sensorStream.SetCharSize(sensorConfig.characterSize);
    sensorStream.SetParity(sensorConfig.parityType);
    sensorStream.SetNumOfStopBits(sensorConfig.stopBits);
}

Toughsonic::~Toughsonic(){
    this->close();
}

void Toughsonic::start(unsigned int updateIntervalMS){
    updateThread = std::thread([&](){
        running = true;
        const auto interval = std::chrono::milliseconds(updateIntervalMS);
        while(running){
            const int bufferSize = 256;
            char inputBuffer[bufferSize];
            ROS_INFO("Reading from stream");
            sensorStream.read(inputBuffer,);
            ROS_INFO("Serial read: %s", read);
            std::this_thread::sleep_for(interval);
        }
        ROS_INFO("Killing thread");
    });
}

void Toughsonic::close(){
    running = false;
    updateThread.join();
    sensorStream.Close();
}