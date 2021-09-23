#include <iostream>
#include "ros/ros.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/susan.h>
#include <pcl/filters/filter.h>
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

class SusanDetector
{
    public:
        SusanDetector()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&SusanDetector::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		
		
		pcl::PCLPointCloud2 pcl_pc2;
		//pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_Nan (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud_with_Nan);
		//pcl::fromPCLPointCloud2(pcl_pc2, *Vcloud);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

		//std::vector<int> indices;
		//pcl::removeNaNFromPointCloud(*cloud_with_Nan, *cloud, indices);
		//std::cout<<"Before: "<< cloud_with_Nan->points.size()<<std::endl;
		//std::cout<<"After: "<< cloud->points.size()<<std::endl;

		double start = std::clock();  
		pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* susan3D = new pcl::SUSANKeypoint<pcl::PointXYZRGB,pcl::PointXYZRGB>;
		susan3D->setInputCloud(cloud_with_Nan);
		susan3D->setNonMaxSupression(true);
		//susan3D->setSearchMethod(tree);
		//susan3D->setRadius(0.5);
		//susan3D->setRadiusSearch(0.005);
		susan3D->compute(*keypoints);
		double end = std::clock();		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>());
		copyPointCloud(*keypoints, *cloud_temp);
		
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;
		std::cout << "Resulting sift points are of size: " << cloud_temp->size () <<std::endl;


		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud_with_Nan, 255, 255, 0);
		viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		viewer.addPointCloud(cloud_with_Nan, "cloud");
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
    SusanDetector PC_obj;
    ros::Rate r(0.01);
    while (ros::ok())
    {
     ros::spinOnce();
     r.sleep();     
     }

    return 0;
}
