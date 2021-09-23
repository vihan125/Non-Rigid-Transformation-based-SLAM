#include <iostream>
#include "ros/ros.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types_conversion.h>

#include <pcl/io/pcd_io.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointField.h"
#include <bits/stdc++.h>

class TjDetector
{
    public:
        TjDetector()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&TjDetector::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		
		
		pcl::PCLPointCloud2 pcl_pc2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_with_Nan (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr original (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *original);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud_with_Nan);

		pcl::PointCloud<pcl::PointXYZI>& data_cloud = *input;

		double start = std::clock(); 

		data_cloud.height = original->height;
		data_cloud.width = original->width;
		std::cout << "h" << original->height <<std::endl;
		std::cout << "w" << original->width <<std::endl;
		data_cloud.resize(original->height * original->width);
		data_cloud.clear();
		
		pcl::PointXYZI point;
		for(int i =0; i< original->size();i++){
			pcl::PointXYZRGBtoXYZI(original->at(i),point);
			data_cloud.push_back(point);

		}

		//Remove Nan points from normal cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		//std::vector<int> indices;
		//pcl::removeNaNFromPointCloud(*cloud_with_Nan, *cloud, indices);

		// Compute the normals
  		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  		normal_estimation.setInputCloud (cloud_with_Nan);
		  
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  		normal_estimation.setSearchMethod (tree);		

 		pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
  		normal_estimation.setKSearch(10);
		normal_estimation.compute (*cloud_with_normals);

		//Ketpoint
		pcl::TrajkovicKeypoint3D<pcl::PointXYZI, pcl::PointXYZI,pcl::Normal> Tj3D;
		Tj3D.setNormals(cloud_with_normals);
		Tj3D.setNumberOfThreads(4);
		Tj3D.setFirstThreshold(10);
		Tj3D.setInputCloud(input);
		//Tj3D.setMethod(4);
		Tj3D.setSecondThreshold(30);
		Tj3D.compute(*keypoints);
		double end = std::clock();		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
		copyPointCloud(*keypoints, *cloud_temp);
		
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;
		std::cout << "Resulting sift points are of size: " << cloud_temp->size () <<std::endl;


		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 0);
		viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		viewer.addPointCloud(cloud, "cloud");
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
    TjDetector PC_obj;
    ros::Rate r(0.05);
    while (ros::ok())
    {
     ros::spinOnce();
     r.sleep();     
     }

    return 0;
}
