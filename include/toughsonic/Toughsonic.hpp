#ifndef TOUGHSONIC_HPP
#define TOUGHSONIC_HPP

#include "ros/ros.h"

class Toughsonic{
public:
    Toughsonic(ros::NodeHandle nh);
    ~Toughsonic();

private:
    ros::NodeHandle n;
};

#endif