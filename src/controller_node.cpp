#include "ros/ros.h"

#include <foilboat_controller/FoilboatController.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "foilboat_controller_node");
    ros::NodeHandle n;

    FoilboatController mFoilboatController(n);
    
    ros::spin();
    return EXIT_SUCCESS;
}