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

            char readChar;
            const int dataLength = 5;
            std::vector<char> measurement;
            measurement.reserve(dataLength);
            for(;;){
                sensorStream.get(readChar);
                if((int)readChar == 13){
                    break;
                }
                measurement.push_back(readChar);
            }

            std::string readString_Original(measurement.begin(), measurement.end());
            if(measurement[0] == '0' && measurement.size() > 1){
                measurement.erase(measurement.begin());
                measurement.shrink_to_fit();
            }

            std::string readString_Sanitized(measurement.begin(), measurement.end());

            double dist = -1;
            try{            
                dist = std::stod(readString_Sanitized);
                dist *= 0.003384;
            } catch(std::invalid_argument &e){
                ROS_ERROR("stod error, invalid argument");
                ROS_ERROR("Error: %s", e.what());
            }

            this->onSensorReadCallback(dist);

            // ROS_INFO("Serial read: %s", readString_Original.c_str());
            // ROS_INFO("Serial read sanitized: (%s)", readString_Sanitized.c_str());
            // ROS_INFO("Distance: %f", dist);
            measurement.clear();
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

void Toughsonic::setSensorReadCallback(std::function<void(int)> callback){
    this->onSensorReadCallback = callback;
}