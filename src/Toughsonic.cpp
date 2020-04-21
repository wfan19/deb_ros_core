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
    sensorStream.Close();
}

void Toughsonic::start(unsigned int updateIntervalMS){
//     running = true;
//     while(running){
//         ROS_INFO("")
//     }
}

