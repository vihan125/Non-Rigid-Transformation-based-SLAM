#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/registration/icp.h>


//msgs
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <bits/stdc++.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Services
#include "registration/getTransform.h"

class RegisterOdom{

    typedef struct
    {
        double x;
        double y;
        double z;
    } Coordinate;


    public:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_rgb;
        pcl::PointCloud<pcl::PointXYZ>::Ptr graph;
		Eigen::Matrix4f globaltransform; 
        registration::getTransform srv;

        RegisterOdom();
        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    private: 
        std::map<float, std::vector<Coordinate>> DeformG = {}; // Deformation graph
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::Publisher publisher2;
        ros::ServiceClient client;
        
};