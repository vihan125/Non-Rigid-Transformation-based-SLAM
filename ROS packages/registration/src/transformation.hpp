#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "registration/getTransform.h"

#include <pcl/point_types.h>

class Transformation{

    public:
        Transformation();
        void calculateTransform(const nav_msgs::Odometry::ConstPtr& odom);
        bool sendTransform(registration::getTransform::Request &req, 
        registration::getTransform::Response &res);
	
    private: 
        static std::map<int, Eigen::Matrix4f> time_data;
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::ServiceServer service;




};