#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <pcl/io/pcd_io.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

//msgs
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <bits/stdc++.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

class FPFHFeatures
{
    public:
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr previous_fpfh_features;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_keypoint_cloud ;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr result;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_rgb;
		Eigen::Matrix4f globaltransform; 

        FPFHFeatures()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&FPFHFeatures::processPointCloud,this);
			this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 1);

			// global clouds
			// global clouds
			previous_fpfh_features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
			previous_keypoint_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			previous_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			result = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
			result_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			globaltransform = Eigen::Matrix4f::Identity();

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

		double start = std::clock(); 
		//Image
		sensor_msgs::Image image_;
		
		//Converting ROS point cloud to Image
		pcl::toROSMsg (*cloud_msg,image_);
		//sensor_msgs::ImageConstPtr img_msg = &image_;
		
		cv::Mat img,filtered_img,keypoints_mat, sharp_mat, contrast_mat, combi_mat;
		cv_bridge::CvImagePtr cvPtr;
		cvPtr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
		cvPtr->image.copyTo(img);

		cv::bilateralFilter(img,filtered_img,15,50,80,cv::BORDER_DEFAULT);
		std::vector<cv::KeyPoint> keypoints;
		cv::Ptr<cv::ORB> detector = cv::ORB::create(150);
		detector->detect(filtered_img, keypoints);
		cv::KeyPoint kk = keypoints[0];
		std::cout << "Resulting key points are of size: " << keypoints.size() <<std::endl; 
		//std::cout << "First Point: " << kk.pt <<std::endl;
		cv::drawKeypoints(filtered_img, keypoints,keypoints_mat, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
		cv::imshow("Keypoints", keypoints_mat);
		cv::waitKey(2);

		
		pcl::PCLPointCloud2 pcl_pc2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud_RGB);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::PointXYZ point;
		for(int i =0; i< keypoints.size();i++){
			double xc =keypoints[i].pt.x;
			double yc =keypoints[i].pt.y;

			point.x = cloud->at(xc,yc).x;
			point.y = cloud->at(xc,yc).y;
			point.z = cloud->at(xc,yc).z;
			keypoint_cloud->push_back(point);

		}

		std::cout<<"Keypoint cloud size: "<< keypoint_cloud->points.size()<<std::endl;
		//Remove Nan points from cloud
		cloud->is_dense = false;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

		//double start = std::clock(); 
		
		// Compute the normals
		pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
  		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		cloud_with_normals->is_dense = false;
		
  		normal_estimation.setInputCloud (cloud);
  		normal_estimation.setSearchMethod (tree);		
		normal_estimation.setKSearch(3);
		normal_estimation.compute (*cloud_with_normals);

		std::cout << "Calculation of Normals done" <<cloud_with_normals->points.size()<<std::endl;

		keypoint_cloud->is_dense = false;
        std::vector<int> indices2;
        pcl::removeNaNFromPointCloud(*keypoint_cloud,*keypoint_cloud, indices2);
		std::cout<<"Remove nan points from keypoints: "<< keypoint_cloud->points.size()<<std::endl;   

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
		// Provide the original point cloud (without normals)
		fpfh_estimation.setInputCloud (keypoint_cloud);
		// Provide the point cloud with normals with no nan
		fpfh_estimation.setInputNormals (cloud_with_normals);
		fpfh_estimation.setSearchSurface(cloud);
		fpfh_estimation.setSearchMethod (tree);

		// Compute features
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
		std::cout << "Calculating FPFH features........" <<std::endl;
		fpfh_estimation.setKSearch(5);
		fpfh_estimation.compute (*fpfh_features);

		std::cout << "Calculated Features size: " << fpfh_features->points.size () <<std::endl;

		double end = std::clock();
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;

		pcl::visualization::PCLVisualizer viewer;

		if(previous_fpfh_features->size()> 0 && previous_keypoint_cloud->size()>0){

			std::cout << "hola" << std::endl;

			double start_r = std::clock();
			// 1. initial alignment of point clouds
			Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
			pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> initSAC;
			pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
			initSAC.setMinSampleDistance(0.1);
			initSAC.setMaxCorrespondenceDistance(0.5);
			initSAC.setMaximumIterations(1000);

			initSAC.setInputCloud(keypoint_cloud);
			initSAC.setSourceFeatures(fpfh_features);

			initSAC.setInputTarget(previous_keypoint_cloud);
			initSAC.setTargetFeatures(previous_fpfh_features);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr initTransCloud (new pcl::PointCloud<pcl::PointXYZ>);
			initSAC.align(*initTransCloud);
			initTransform = initSAC.getFinalTransformation();
			
			double end_r = std::clock();
			double time_r = double(end_r-start_r) / double(CLOCKS_PER_SEC);
			std::cout << "done init :" <<time_r<<" seconds"<<std::endl;
			// 2. Fine alignment using ICP
			Eigen::Matrix4f finalTransform = Eigen::Matrix4f::Identity();
			pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
			icp.setMaxCorrespondenceDistance(0.1);
			icp.setRANSACOutlierRejectionThreshold(0.2);
			icp.setTransformationEpsilon(0.01);
			icp.setMaximumIterations(10000);
			icp.setInputCloud(initTransCloud);
			icp.setInputTarget(previous_keypoint_cloud);

			pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
			icp.align(*finalCloud);
			finalTransform = icp.getFinalTransformation() * initTransform;
			
			double end2_r = std::clock();
			double time2_r = double(end2_r-end_r) / double(CLOCKS_PER_SEC);
			std::cout << "done init :" <<time2_r<<" seconds"<<std::endl;
			//global transform --
			globaltransform = globaltransform * finalTransform;
			std::cout << "Global Transform:" << std::endl<< globaltransform << std::endl;

			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::transformPointCloud(*cloud, *transformed_source, globaltransform);
			pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);

			viewer.setBackgroundColor(0, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(transformed_source, 150, 80, 80);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_keypoints(keypoint_cloud, 255, 0, 0);

			viewer.addPointCloud<pcl::PointXYZ>(transformed_source, handler_source_cloud, "source_cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
			viewer.addPointCloud<pcl::PointXYZ>(keypoint_cloud, handler_source_keypoints, "source_keypoints");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_keypoints");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_cloud(result, 80, 150, 80);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_keypoints(previous_keypoint_cloud, 0, 255, 0);

			viewer.addPointCloud<pcl::PointXYZ>(result, handler_target_cloud, "target_cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
			viewer.addPointCloud<pcl::PointXYZ>(previous_keypoint_cloud, handler_target_keypoints, "target_keypoints");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target_keypoints");
			viewer.addCorrespondences<pcl::PointXYZ>(previous_keypoint_cloud,keypoint_cloud, *correspondences_filtered, "correspondences");

			previous_fpfh_features->clear();
			previous_keypoint_cloud->clear();
			pcl::copyPointCloud(*fpfh_features,*previous_fpfh_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);
			result->operator+=(*transformed_source);
			result_rgb->operator+=(*transformed_source_RGB);
			std::cout<<"cloud size :"<<cloud->width<<"x"<<cloud->height<<std::endl;
			std::cout<<"Concatanated point cloud size :"<<result->width<<"x"<<result->height<<std::endl;

			publisher.publish(result_rgb);
		}
		else{
			std::cout << "first_time" << std::endl;
			pcl::copyPointCloud(*fpfh_features,*previous_fpfh_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);
			pcl::copyPointCloud(*cloud,*result);
			pcl::copyPointCloud(*cloud_RGB,*result_rgb);
		}

		while(!viewer.wasStopped())
		{
			viewer.spinOnce();
		}

		// while (!ICPView.wasStopped())
		// {

		// 	ICPView.spinOnce();
		// }

		// viewer.close();
		// ICPView.close();

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
    FPFHFeatures PC_obj;
    //ros::spin();
    ros::Rate r(0.05);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}