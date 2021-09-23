#include <iostream>
#include "ros/ros.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/brisk_2d.h>
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

class BriskDetector
{
    public:
        BriskDetector()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&BriskDetector::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		
		
		pcl::PCLPointCloud2 pcl_pc2;
		//pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr view (new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *view);
		//pcl::fromPCLPointCloud2(pcl_pc2, *Vcloud);
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	
		double start = std::clock(); 
		pcl::BriskKeypoint2D<pcl::PointXYZRGBA> brisk;
		brisk.setThreshold (60);
		brisk.setOctaves (4);
		brisk.setInputCloud (cloud);  	
		brisk.compute(*keypoints);
		
		double end = std::clock();		

		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
		// copyPointCloud(*keypoints, *cloud_temp);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& data_cloud = *cloud_temp;

		data_cloud.height = keypoints->height;
		data_cloud.width = keypoints->width;
		std::cout << "h" << keypoints->height <<std::endl;
		std::cout << "w" << keypoints->width <<std::endl;
		data_cloud.resize(keypoints->height * keypoints->width);
		data_cloud.clear();
		
		pcl::PointXYZ point;
		for(int i =0; i< keypoints->size();i++){
			double xc =keypoints->at(i).x;
			double yc =keypoints->at(i).y;

			point.x = cloud->at(xc,yc).x;
			point.y = cloud->at(xc,yc).y;
			point.z = cloud->at(xc,yc).z;
			data_cloud.push_back(point);

		}
		
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;
		std::cout << "Resulting sift points are of size: " << keypoints->size () <<std::endl;


		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (view, 255, 255, 0);
		viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		viewer.addPointCloud(view, "cloud");
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
    BriskDetector PC_obj;
    ros::Rate r(0.05);
    while (ros::ok())
    {
     ros::spinOnce();
     r.sleep();     
     }

    return 0;
}
