#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>

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

class SHOTFeatures
{
    public:

		pcl::PointCloud<pcl::SHOT352>::Ptr previous_shot_features;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_keypoint_cloud ;
		pcl::PointCloud<pcl::PointXYZ>::Ptr result;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_rgb;
		Eigen::Matrix4f globaltransform; 

        SHOTFeatures()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&SHOTFeatures::processPointCloud,this);
            this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 1);

			// global clouds
			previous_shot_features = pcl::PointCloud<pcl::SHOT352>::Ptr (new pcl::PointCloud<pcl::SHOT352>);
			previous_keypoint_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
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
		normal_estimation.setRadiusSearch(0.03);
		normal_estimation.compute (*cloud_with_normals);

		pcl::PointCloud<pcl::Normal>::Ptr normals_Nan (new pcl::PointCloud<pcl::Normal>);
		normals_Nan->is_dense = false;
		normals_Nan->resize(cloud_with_normals->size());
		int count =0 ;
		for(int i =0 ; i<cloud_with_normals->points.size () ; i++){

			if(isnan(cloud_with_normals->points[i].normal_x)){
				continue;
			}else{
				normals_Nan->at(i)=cloud_with_normals->at(i);
			}	
		}
		
		std::cout << "Calculation of Normals done" <<normals_Nan->points.size()<<std::endl;

		keypoint_cloud->is_dense = false;
        std::vector<int> indices2;
        pcl::removeNaNFromPointCloud(*keypoint_cloud,*keypoint_cloud, indices2);
		std::cout<<"Remove nan points from keypoints: "<< keypoint_cloud->points.size()<<std::endl;   

		pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_estimation;
		// Provide the original point cloud (without normals)
		shot_estimation.setInputCloud (keypoint_cloud);
		// Provide the point cloud with normals with no nan
		shot_estimation.setInputNormals (normals_Nan);
		shot_estimation.setSearchSurface(cloud);
		shot_estimation.setSearchMethod (tree);

		// Compute features
		pcl::PointCloud<pcl::SHOT352>::Ptr shot_features (new pcl::PointCloud<pcl::SHOT352>);
		std::cout << "Calculating SHOT features........" <<std::endl;
		//shot_estimation.setKSearch(10);
        shot_estimation.setRadiusSearch(0.02);
		shot_estimation.compute (*shot_features);

		// Display and retrieve the shape context descriptor vector for the 0th point.
		std::cout << "Calculated Features size: " << shot_features->points.size () <<std::endl;
		
		//pcl::FPFHSignature33 descriptor = (*fpfh_features)[0];
		//std::cout << descriptor << std::endl;
		double end = std::clock();
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;

		// pcl::visualization::PCLVisualizer viewer("PCL Viewer");


		std::cout << previous_shot_features->size() << std::endl;

		if(previous_shot_features->size()> 0 && previous_keypoint_cloud->size()>0){
			std::cout << "hola" << std::endl;

            // correspondence rejection pipeline
			double start_r = std::clock();

			// 1. correspondence estimation using equalent features
			pcl::CorrespondencesPtr correspondences (new pcl::Correspondences());
			pcl::Correspondence c;

			for(int i = 0; i<shot_features->size(); i++){

				for(int j = 0; j < previous_shot_features->size(); j++){
                    if(isnan(shot_features->at(i).descriptor[0]) || isnan(previous_shot_features->at(j).descriptor[0])){
                        continue;
                    }else{
                        if(*shot_features->at(i).descriptor == *previous_shot_features->at(j).descriptor){

                            c.index_query = i;
                            c.index_match = j;
                            c.distance = pcl::squaredEuclideanDistance(keypoint_cloud->at(i),previous_keypoint_cloud->at(j));
                            correspondences->push_back(c);
                        }
                    }
				}
			}
			std::cout << "After feature matching :" << correspondences->size()<<std::endl;

			// 2. Reject correspondances which have dostance greater than median distance
			pcl::CorrespondencesPtr correspondences_distance (new pcl::Correspondences());
			pcl::registration::CorrespondenceRejectorMedianDistance distanceRejector;
			distanceRejector.setInputCorrespondences(correspondences);
			distanceRejector.setMedianFactor(0.1);
			distanceRejector.getCorrespondences(*correspondences_distance);
			std::cout << "After distant filtering :" << correspondences_distance->size()<<std::endl;

			// 3. Correspondance rejection RANSAC
			Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
			pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
			rejector_sac.setInputSource(keypoint_cloud);
			rejector_sac.setInputTarget(previous_keypoint_cloud);
			rejector_sac.setInlierThreshold(0.05); // distance in m, not the squared distance
			rejector_sac.setMaximumIterations(1000000);
			rejector_sac.setRefineModel(false);
			rejector_sac.setInputCorrespondences(correspondences_distance);
			rejector_sac.getCorrespondences(*correspondences_filtered);

            std::cout << "Final correspondences :" << correspondences_filtered->size()<<std::endl;

			// for(int j = 0; j < correspondences_filtered->size(); j++){
			// 	std::cout<<correspondences_filtered->at(j)<<std::endl; 
			// };
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr initTransCloud (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> trans_est;
			trans_est.estimateRigidTransformation (*keypoint_cloud,*previous_keypoint_cloud, *correspondences_filtered, initTransform);
            pcl::transformPointCloud(*keypoint_cloud, *initTransCloud, initTransform);

			// 2. Fine alignment using ICP
			Eigen::Matrix4f finalTransform = Eigen::Matrix4f::Identity();
			pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
			icp.setMaxCorrespondenceDistance(0.05);
			icp.setRANSACOutlierRejectionThreshold(0.2);
			icp.setTransformationEpsilon(0.01);
			icp.setMaximumIterations(10000);
			icp.setInputCloud(initTransCloud);
			icp.setInputTarget(previous_keypoint_cloud);

			pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
			icp.align(*finalCloud);
			finalTransform = icp.getFinalTransformation() * initTransform;

			double end_r = std::clock();
			double time2_reg = double(end_r-start_r) / double(CLOCKS_PER_SEC);
			std::cout << "done registration :" <<time2_reg<<" seconds"<<std::endl;

			//global transform --
			globaltransform = globaltransform * finalTransform;
			std::cout << "Global Transform:" << std::endl<< globaltransform << std::endl;

			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::transformPointCloud(*cloud, *transformed_source, globaltransform);
			pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);

			// viewer.setBackgroundColor(0, 0, 0);
			// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(transformed_source, 150, 80, 80);
			// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_keypoints(keypoint_cloud, 255, 0, 0);

			// viewer.addPointCloud<pcl::PointXYZ>(transformed_source, handler_source_cloud, "source_cloud");
			// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
			// viewer.addPointCloud<pcl::PointXYZ>(keypoint_cloud, handler_source_keypoints, "source_keypoints");
			// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_keypoints");
			// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_cloud(result, 80, 150, 80);
			// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_keypoints(previous_keypoint_cloud, 0, 255, 0);

			// viewer.addPointCloud<pcl::PointXYZ>(result, handler_target_cloud, "target_cloud");
			// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
			// viewer.addPointCloud<pcl::PointXYZ>(previous_keypoint_cloud, handler_target_keypoints, "target_keypoints");
			// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target_keypoints");
			// viewer.addCorrespondences<pcl::PointXYZ>(previous_keypoint_cloud,keypoint_cloud, *correspondences_filtered, "correspondences");

			previous_shot_features->clear();
			previous_keypoint_cloud->clear();
			pcl::copyPointCloud(*shot_features,*previous_shot_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);
			result->operator+=(*transformed_source);
			result_rgb->operator+=(*transformed_source_RGB);
			std::cout<<"cloud size :"<<cloud->width<<"x"<<cloud->height<<std::endl;
			std::cout<<"Concatanated point cloud size :"<<result->width<<"x"<<result->height<<std::endl;

			publisher.publish(result_rgb);


		}
		else{
			std::cout << "first_time" << std::endl;
			pcl::copyPointCloud(*shot_features,*previous_shot_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);
			pcl::copyPointCloud(*cloud,*result);
			pcl::copyPointCloud(*cloud_RGB,*result_rgb);
		}
		// while(!viewer.wasStopped())
		// {
		// 	viewer.spinOnce();
		// }

		// while (!ICPView.wasStopped())
		// {

		// 	ICPView.spinOnce();
		// }

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
    SHOTFeatures PC_obj;
    //ros::spin();
    ros::Rate r(0.05);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}