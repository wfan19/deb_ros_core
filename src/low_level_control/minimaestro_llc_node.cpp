#include <string>
#include <map>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <fcs_ros_deb/FoilboatControl.h>

using namespace std;

ros::Publisher left_wing_pub, right_wing_pub;

double flap_min, flap_max;

void onControl(const fcs_ros_deb::FoilboatControl::ConstPtr controlPtr)
{
    double left_flap = controlPtr->leftFoil;
    double right_flap = controlPtr->rightFoil;

    // Map left and right flap data from radian angle range (-10deg - 30deg for example) to 0 - 1.

    std_msgs::Float64 left_msg, right_msg;
    left_msg.data = left_flap;
    right_msg.data = right_flap;

    left_msg.data -= flap_min;
    right_msg.data -= flap_min;

    left_msg.data = left_msg.data / abs(flap_max - flap_min);
    right_msg.data = right_msg.data / abs(flap_max - flap_min);

    left_msg.data = 1 - left_msg.data;

    ROS_INFO("[FoilboatLLC]: Foil target angles: [%f, %f], scaled output: [%f, %f]", left_flap, right_flap, left_msg.data, right_msg.data);
    left_wing_pub.publish(left_msg);
    right_wing_pub.publish(right_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "foilboat_llc");
    ros::NodeHandle n;
    
    string left_wing_name, right_wing_name, control_name;
    if(n.getParam("minimaestro_llc/left_wing", left_wing_name))
    {
        left_wing_pub = n.advertise<std_msgs::Float64>(left_wing_name, 100);
        ROS_INFO("Left wing publisher initialized");
    }

    if(n.getParam("minimaestro_llc/right_wing", right_wing_name))
    {
        right_wing_pub = n.advertise<std_msgs::Float64>(right_wing_name, 100);
        ROS_INFO("Right wing publisher initialized");
    }

    ros::Subscriber control_sub;
    if(n.getParam("minimaestro_llc/control", control_name))
    {
        control_sub = n.subscribe(control_name, 1000, onControl);
        ROS_INFO("Control subscriber initialized");
    }

    map<string, double> range_map;
    if(n.getParam("minimaestro_llc/flap_range", range_map))
    {
        map<string, double>::iterator range_iterator;
        for(range_iterator = range_map.begin(); range_iterator != range_map.end(); range_iterator++)
        {
            if(range_iterator->first == "min")
                flap_min = range_iterator->second;
            else if(range_iterator->first == "max")
                flap_max = range_iterator->second;
            else
                ROS_ERROR("[FoilboatLLC] unknown param detected");
        }
        ROS_INFO("Flap range gotten: [%f, %f]", flap_min, flap_max);
    }

    ros::spin();
    return 0;
}