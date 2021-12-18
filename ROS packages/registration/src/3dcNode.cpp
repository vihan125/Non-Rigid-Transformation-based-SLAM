#include <iostream>
#include "ros/ros.h"
#include "reg_3dc.hpp"

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"observe_PointCloud");
    std::cout << "Process_PointCloud under observation" << std::endl;

    // handle ROS communication events
    DSCFeatures PC_obj;
    //ros::spin();
    ros::Rate r(0.25);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}