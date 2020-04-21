#include "../include/toughsonic/Toughsonic.hpp"

Toughsonic::Toughsonic(ros::NodeHandle nh, Toughsonic::SensorConfig sensorConfig){
    this->n = nh;
    sensorStream.Open(sensorConfig.filename);
}

Toughsonic::~Toughsonic(){
}

