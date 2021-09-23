#include <iostream>
#include "ros/ros.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointField.h"
#include <bits/stdc++.h>

class ISSDetector
{
    public:
        ISSDetector()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&ISSDetector::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		
		
		pcl::PCLPointCloud2 pcl_pc2;
		//pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *Vcloud);
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	
		double start = std::clock();  
		pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;
		iss_detector.setSearchMethod (tree);
		iss_detector.setSalientRadius (6*0.005);
		iss_detector.setNonMaxRadius (4*0.005);
		iss_detector.setThreshold21 (0.5);
		iss_detector.setThreshold32 (0.5);
		iss_detector.setMinNeighbors (10);
		iss_detector.setNumberOfThreads (4);
		iss_detector.setInputCloud (cloud);
		iss_detector.compute (*keypoints); 
		double end = std::clock();		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
		copyPointCloud(*keypoints, *cloud_temp);
		
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;
		std::cout << "Resulting sift points are of size: " << cloud_temp->size () <<std::endl;


		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (Vcloud, 255, 255, 0);
		viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		viewer.addPointCloud(Vcloud, "cloud");
		viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		while(!viewer.wasStopped())
		{
			viewer.spinOnce();
		}

		std::cout<<"***********************************************************"<<std::endl;
	   	
        }


    private: 
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
};

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"observe_PointCloud");
    std::cout << "Process_PointCloud under observation" << std::endl;

    // handle ROS communication events
    ISSDetector PC_obj;
    ros::Rate r(0.025);
    while (ros::ok())
    {
     ros::spinOnce();
     r.sleep();     
     }

    return 0;
}
