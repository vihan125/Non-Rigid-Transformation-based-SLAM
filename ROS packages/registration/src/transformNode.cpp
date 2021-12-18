#include <iostream>
#include "ros/ros.h"
#include "transformation.hpp"

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"Transform_storage");
    std::cout << "Transformation calculating.." << std::endl;

    // handle ROS communication events
    Transformation T;
    //ros::spin();
    ros::Rate r(1);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}