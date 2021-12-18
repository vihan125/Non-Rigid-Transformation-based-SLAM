#include <iostream>
#include "ros/ros.h"
#include "tf_transform.hpp"

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"Registration_TF");
    std::cout << "Register point clouds from odometry data" << std::endl;

    // Intializing Tflistner and buffer
    buffer = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buffer);

    // handle ROS communication events
    RegisterTF TF;
    //ros::spin();
    ros::Rate r(10);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;

}