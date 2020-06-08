#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <foilboat_controller/FoilboatControl.h>

ros::Publisher left_wing_pub, right_wing_pub;

void onControl(const foilboat_controller::FoilboatControl::ConstPtr controlPtr)
{
    double left_flap = controlPtr->leftFoil;
    double right_flap = controlPtr->rightFoil;

    std_msgs::Float64 left_msg, right_msg;
    left_msg.data = left_flap;
    right_msg.data = right_flap;

    ROS_INFO("Publishing control states %f and %f", left_flap, right_flap);
    left_wing_pub.publish(left_msg);
    right_wing_pub.publish(right_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "foilboat_llc");
    ros::NodeHandle n;
    
    std::string left_wing_name, right_wing_name, control_name;
    if(n.getParam("low_level_control/left_wing", left_wing_name))
    {
        left_wing_pub = n.advertise<std_msgs::Float64>(left_wing_name, 100);
        ROS_INFO("Left wing publisher initialized");
    }

    if(n.getParam("low_level_control/right_wing", right_wing_name))
    {
        right_wing_pub = n.advertise<std_msgs::Float64>(right_wing_name, 100);
        ROS_INFO("Right wing publisher initialized");
    }

    ros::Subscriber control_sub;
    if(n.getParam("low_level_control/control", control_name))
    {
        control_sub = n.subscribe(control_name, 1000, onControl);
        ROS_INFO("Control subscriber initialized");
    }

    ros::spin();
    return 0;
}