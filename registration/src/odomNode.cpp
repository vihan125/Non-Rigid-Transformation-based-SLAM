#include <iostream>
#include "ros/ros.h"
#include "odom.hpp"

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"Registration_odometry");
    std::cout << "Register point clouds from odometry data" << std::endl;

    // handle ROS communication events
    RegisterOdom ROdom;
    //ros::spin();
    ros::Rate r(1);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}