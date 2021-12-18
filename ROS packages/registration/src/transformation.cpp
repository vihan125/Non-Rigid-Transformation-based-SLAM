#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <pcl/point_types.h>
#include "transformation.hpp"



Transformation::Transformation(){
	this->subscriber = this->nh.subscribe<nav_msgs::Odometry>("/odom",5,&Transformation::calculateTransform,this);
	this->service = this->nh.advertiseService("get_Transform",&Transformation::sendTransform,this);
}


std::map<int, Eigen::Matrix4f> Transformation::time_data;

void Transformation::calculateTransform(const nav_msgs::Odometry::ConstPtr& odom){

	int id = odom->header.stamp.sec;

	/* axis mapping.
		Transformation of point cloud should happrn according to movement of robot,
		Robots x == point cloud z
		Robots y == point cloud x
		Robots z == point cloud y
	*/

	std::cout<<id<<std::endl;

	if(time_data.find(id-180) != time_data.end())
		time_data.erase(id-180);

	float x = odom->pose.pose.position.x;
	float y = odom->pose.pose.position.y;
	float z = odom->pose.pose.position.z;

	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
	translation(0,3) = -y;
	translation(1,3) = z;
	translation(2,3) = x;

	//calculating rotation around x axis in robot
	float x_a = odom->pose.pose.orientation.x;
	float y_a = odom->pose.pose.orientation.y;
	float z_a = odom->pose.pose.orientation.z;
	float w_a = odom->pose.pose.orientation.w;

	float sinr_cosp = 2 * (w_a * x_a + y_a*z_a);
	float cosr_cosp = 1 - 2 * (x_a * x_a + y_a * y_a);
	float angle_x = std::atan2(sinr_cosp,cosr_cosp);

	float cos_x_a = std::cos(angle_x);
	float sin_x_a = std::sin(angle_x);

	Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity(); // rotation around z -axis in cloud
	rotation_z(0,0) = cos_x_a;
	rotation_z(0,1) = -sin_x_a;
	rotation_z(1,0) = sin_x_a;
	rotation_z(1,1) = cos_x_a;

	//calculating rotation around y axis in robot
	float sinp = 2 * (w_a * y_a - z_a*x_a);
	float angle_y;
	if (std::abs(sinp)>= 1)
		angle_y = std::copysign(M_PI / 2, sinp);
	else
		angle_y = std::asin(sinp);
		

	float cos_y_a = std::cos(angle_y);
	float sin_y_a = std::sin(angle_y);

	Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity(); // rotation around x -axis in cloud
	rotation_x(1,1) = cos_y_a;
	rotation_x(1,2) = -sin_y_a;
	rotation_x(2,1) = sin_y_a;
	rotation_x(2,2) = cos_y_a;

	//calculating rotation around z axis in robot
	float siny_cosp = 2 * (w_a * z_a + x_a*y_a);
	float cosy_cosp = 1 - 2 * (y_a * y_a + z_a * z_a);
	float angle_z = std::atan2(siny_cosp,cosy_cosp);

	float cos_z_a = std::cos(angle_z);
	float sin_z_a = std::sin(angle_z);

	Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity(); // rotation around y -axis in cloud
	rotation_y(0,0) = cos_z_a;
	rotation_y(0,2) = -sin_z_a;
	rotation_y(2,0) = sin_z_a;
	rotation_y(2,2) = cos_z_a;

	Eigen::Matrix4f currenttransform;
	
	currenttransform = translation * rotation_x * rotation_y * rotation_z;

	time_data[id] = currenttransform;
	std::cout<<"size :"<<time_data.size()<<std::endl;
}

// Eigen::Matrix4f Transformation::getTransform (int date){
// 	std::cout<<"time :"<<date<<std::endl;
// 	std::cout<<"size :"<<time_data.size()<<std::endl;
// 	Eigen::Matrix4f result;
// 	if(time_data.find(date) == time_data.end()){
// 		result = Eigen::Matrix4f::Identity();
// 		std::cout<<"not found"<<std::endl;
// 	}
// 	else
// 		result = time_data.at(date);

// 	return result;
// }

bool Transformation::sendTransform(registration::getTransform::Request &req, 
registration::getTransform::Response &res){

	int date = req.date;
	std::cout<<"time :"<<date<<std::endl;
	std::cout<<"size :"<<time_data.size()<<std::endl;
	Eigen::Matrix4f result;
	if(time_data.find(date) == time_data.end()){
		result = Eigen::Matrix4f::Identity();
		std::cout<<"not found"<<std::endl;
	}
	else
		result = time_data.at(date);

	res.matrix.entry_0_0 = result(0,0);
	res.matrix.entry_0_1 = result(0,1);
	res.matrix.entry_0_2 = result(0,2);
	res.matrix.entry_0_3 = result(0,3);

	res.matrix.entry_1_0 = result(1,0);
	res.matrix.entry_1_1 = result(1,1);
	res.matrix.entry_1_2 = result(1,2);
	res.matrix.entry_1_3 = result(1,3);

	res.matrix.entry_2_0 = result(2,0);
	res.matrix.entry_2_1 = result(2,1);
	res.matrix.entry_2_2 = result(2,2);
	res.matrix.entry_2_3 = result(2,3);

	res.matrix.entry_3_0 = result(3,0);
	res.matrix.entry_3_1 = result(3,1);
	res.matrix.entry_3_2 = result(3,2);
	res.matrix.entry_3_3 = result(3,3);

	ROS_INFO("request: time(s)=%ld,", (long int)req.date);
	ROS_INFO("sending back response:");
	std::cout<<result<<std::endl;

	return true;

};




