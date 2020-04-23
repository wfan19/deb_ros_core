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
            // ROS_INFO("Reading from stream");

            // const int bufferSize = 10;
            // char inputBuffer[bufferSize];
            // sensorStream.read(inputBuffer,bufferSize);
            // std::string readString;
            // readString = inputBuffer;
            char readChar;
            bool reading = true;
            const int dataLength = 5;
            char measurement[dataLength];
            int counter = 0;
            for(;;){
                sensorStream.get(readChar);
                if((int)readChar == 13){
                    break;
                }
                measurement[counter] = readChar;
                counter++;
                if(counter >= 5){
                    break;
                }
            }
            std::string readStringOriginal = measurement;
            
            // float dist = std::stoi(readString);
            // dist *= 0.003384;

            // std::string inputBuffer = "";
            // std::getline(sensorStream, inputBuffer);

            ROS_INFO("Serial read: %s", readStringOriginal.c_str());
            // ROS_INFO("Distance: %d", dist);
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