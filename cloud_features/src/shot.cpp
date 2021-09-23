#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
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

class SHOTFeatures
{
    public:

		pcl::PointCloud<pcl::SHOT352>::Ptr previous_shot_features;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_keypoint_cloud ;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr result;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_rgb;
		Eigen::Matrix4f globaltransform; 

        SHOTFeatures()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&SHOTFeatures::processPointCloud,this);

			// global clouds
			previous_shot_features = pcl::PointCloud<pcl::SHOT352>::Ptr (new pcl::PointCloud<pcl::SHOT352>);
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
		
		cv::Mat img, keypoints_mat, sharp_mat, contrast_mat, combi_mat;
		cv_bridge::CvImagePtr cvPtr;
		cvPtr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
		cvPtr->image.copyTo(img);

		std::vector<cv::KeyPoint> keypoints;
		cv::Ptr<cv::ORB> detector = cv::ORB::create(200);
		detector->detect(img, keypoints);
		cv::KeyPoint kk = keypoints[0];
		std::cout << "Resulting key points are of size: " << keypoints.size() <<std::endl; 
		//std::cout << "First Point: " << kk.pt <<std::endl;
		cv::drawKeypoints(img, keypoints,keypoints_mat, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
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

		std::cout << "Calculation of Normals done" <<cloud_with_normals->points.size()<<std::endl;

		keypoint_cloud->is_dense = false;
        std::vector<int> indices2;
        pcl::removeNaNFromPointCloud(*keypoint_cloud,*keypoint_cloud, indices2);
		std::cout<<"Remove nan points from keypoints: "<< keypoint_cloud->points.size()<<std::endl;   

		pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot_estimation;
		// Provide the original point cloud (without normals)
		shot_estimation.setInputCloud (keypoint_cloud);
		// Provide the point cloud with normals with no nan
		shot_estimation.setInputNormals (cloud_with_normals);
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

		pcl::PointCloud<pcl::SHOT352>::Ptr shot_features_Nan (new pcl::PointCloud<pcl::SHOT352>);
		int count =0 ;
		for(int i =0 ; i<shot_features->points.size () ; i++){
			if(isnan(shot_features->points[i].descriptor[0])){
				continue;
			}else{
				shot_features_Nan->push_back(shot_features->at(i));
			}	
		}

		std::cout << "New features :" <<shot_features_Nan->points.size()<<std::endl;
		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoint_cloud, 0, 255, 0);
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud_RGB, 255, 255, 0);
		//viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		//viewer.addPointCloud(cloud_RGB, "cloud");
		// viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud,cloud_with_normals, 25, 0.15,"normals"); 
		// viewer.addPointCloud(keypoint_cloud, keypoints_color_handler, "keypoints");

		// pcl::visualization::PCLVisualizer ICPView("ICP Viewer");


		std::cout << previous_shot_features->size() << std::endl;

		if(previous_shot_features->size()> 0 && previous_keypoint_cloud->size()>0){
			std::cout << "hola" << std::endl;
			
			pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
    		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    		est.setInputSource(previous_shot_features);
    		est.setInputTarget(shot_features_Nan);
    		est.determineCorrespondences(*correspondences);

			pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
			pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
			corr_rej_one_to_one.setInputCorrespondences(correspondences);
			corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);

			// Correspondance rejection RANSAC

			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
			pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
			rejector_sac.setInputSource(previous_keypoint_cloud);
			rejector_sac.setInputTarget(keypoint_cloud);
			rejector_sac.setInlierThreshold(0.01); // distance in m, not the squared distance
			// rejector_sac.setMaximumIterations(1000);
			// rejector_sac.setRefineModel(false);
			// rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);
			// rejector_sac.getCorrespondences(*correspondences_filtered);
			// correspondences.swap(correspondences_filtered);
			rejector_sac.getRemainingCorrespondences(*correspondences_result_rej_one_to_one,*correspondences_filtered);
			//rejector_sac.setInputCorrespondences(correspondences_filtered);
			std::cout << correspondences_result_rej_one_to_one->size() << " vs. " << correspondences_filtered->size() << std::endl;
			transform = rejector_sac.getBestTransformation(); // Transformation Estimation method 1

			// Transformation Estimation method 2
			//pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
			//transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);
			std::cout << "Estimated Transform:" << std::endl<< transform << std::endl;

			globaltransform = globaltransform * transform;

			std::cout << "Global Transform:" << std::endl<< globaltransform << std::endl;
			// / refinement transform source using transformation matrix ///////////////////////////////////////////////////////

			
		// 	// / refinement transform source using transformation matrix ///////////////////////////////////////////////////////

			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*cloud, *transformed_source, globaltransform);
			pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);

			viewer.setBackgroundColor(0, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(transformed_source, 150, 80, 80);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_keypoints(previous_keypoint_cloud, 255, 0, 0);

			viewer.addPointCloud<pcl::PointXYZ>(transformed_source, handler_source_cloud, "source_cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
			viewer.addPointCloud<pcl::PointXYZ>(keypoint_cloud, handler_source_keypoints, "source_keypoints");

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_cloud(result, 80, 150, 80);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_keypoints(keypoint_cloud, 0, 255, 0);

			viewer.addPointCloud<pcl::PointXYZ>(result, handler_target_cloud, "target_cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
			viewer.addPointCloud<pcl::PointXYZ>(previous_keypoint_cloud, handler_target_keypoints, "target_keypoints");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target_keypoints");
			viewer.addCorrespondences<pcl::PointXYZ>(previous_keypoint_cloud, keypoint_cloud, *correspondences_filtered, "correspondences");


			previous_shot_features->clear();
			previous_keypoint_cloud->clear();
			pcl::copyPointCloud(*shot_features_Nan,*previous_shot_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);

			result->operator+=(*transformed_source);
			result_rgb->operator+=(*transformed_source_RGB);
			std::cout<<"cloud size :"<<cloud->width<<"x"<<cloud->height<<std::endl;
			std::cout<<"Concatanated point cloud size :"<<result->width<<"x"<<result->height<<std::endl;
			// ICPView.setBackgroundColor(0, 0, 0);
			// //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb_cloud(result_rgb, 255, 255, 0);
			// ICPView.addPointCloud<pcl::PointXYZRGB>(result_rgb, "result_cloud");

		// 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		// 	icp.setInputSource(transformed_source);
		// 	icp.setInputTarget(cloud);
		// 	icp.align(*final_output);
		// 	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		// 	std::cout << icp.getFinalTransformation() << std::endl;

		// 	ICPView.addPointCloud<pcl::PointXYZ>(final_output, handler_source_cloud, "Final_cloud");
		// 	ICPView.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_keypoints");
		}
		else{
			std::cout << "first_time" << std::endl;
			pcl::copyPointCloud(*shot_features_Nan,*previous_shot_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);
			pcl::copyPointCloud(*cloud,*previous_cloud);
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
    ros::Rate r(0.01);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}