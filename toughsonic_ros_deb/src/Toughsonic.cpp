#include "ros/ros.h"

#include <toughsonic/Toughsonic.hpp>

Toughsonic::Toughsonic(Toughsonic::SensorConfig sensorConfig)
{
  sensorStream.Open(sensorConfig.filename);
  sensorStream.SetBaudRate(sensorConfig.baudRate);
  sensorStream.SetCharSize(sensorConfig.characterSize);
  sensorStream.SetParity(sensorConfig.parityType);
  sensorStream.SetNumOfStopBits(sensorConfig.stopBits);
}

Toughsonic::~Toughsonic()
{
}

double Toughsonic::read()
{
  char readChar;
  std::vector<char> measurement;
  for(;;) // Keep looking for new lines
  {
    sensorStream.get(readChar);
    if((int)readChar == 13) // Detect new line
    {
      // Begin reading new line
      for(;;)
      {
        sensorStream.get(readChar);
        if((int)readChar != 13) // Check if we have reached the end of the line
        {
          measurement.push_back(readChar);
        }
        else
        {
          break;
        }
      }
      break;
    }
  };

  if(measurement[0] == '0' && measurement.size() > 1)
  {
    measurement.erase(measurement.begin());
    measurement.shrink_to_fit();
  }

  std::string readString_Sanitized(measurement.begin(), measurement.end());

  double dist = -1;
  try
  {            
    dist = std::stod(readString_Sanitized);
    dist *= 0.003384;
  } 
  catch(std::invalid_argument &e)
  {
    ROS_ERROR("Failed to convert read string to double!");
    return -1;
  }
  return dist;
}