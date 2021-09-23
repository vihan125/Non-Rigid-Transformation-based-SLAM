#include <iostream>
#include "ros/ros.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/agast_2d.h>
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

class AgastDetector
{
    public:
        AgastDetector()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&AgastDetector::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		
		
		pcl::PCLPointCloud2 pcl_pc2;
		//pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Vcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *Vcloud);
		
	
		  
		double start = std::clock();  
		// Estimate the sift interest points using Intensity values from RGB values
		pcl::AgastKeypoint2D<pcl::PointXYZRGBA> agast;
		pcl::PointCloud<pcl::PointUV>::Ptr result(new pcl::PointCloud<pcl::PointUV>);
		agast.setThreshold (20);
		agast.setInputCloud (cloud);
		agast.compute(*result);
		double end = std::clock();		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& data_cloud = *cloud_temp;

		data_cloud.height = result->height;
		data_cloud.width = result->width;
		std::cout << "h" << result->height <<std::endl;
		std::cout << "w" << result->width <<std::endl;
		data_cloud.resize(result->height * result->width);
		data_cloud.clear();
		
		pcl::PointXYZ point;
		for(int i =0; i< result->size();i++){
			double xc =result->at(i).u;
			double yc =result->at(i).v;

			point.x = cloud->at(xc,yc).x;
			point.y = cloud->at(xc,yc).y;
			point.z = cloud->at(xc,yc).z;
			data_cloud.push_back(point);
		}
		
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken" << time_taken <<std::endl;
		std::cout << "Resulting AGAST points are of size: " << result->size () <<std::endl;


		//for(int i =0; i<cloud_temp->size();i++){
			//std::cout << "x,y,z: " << cloud_temp->at(i).x <<" , " << cloud_temp->at(i).y <<" , "<< cloud_temp->at(i).z <<std::endl;
		//}


		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp,0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (Vcloud,255, 255, 0);
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
    AgastDetector PC_obj;
    ros::Rate r(0.01);
    while (ros::ok())
    {
     ros::spinOnce();
     r.sleep();     
     }

    return 0;
}
