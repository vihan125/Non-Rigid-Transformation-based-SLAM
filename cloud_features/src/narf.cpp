#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>

#include <pcl/features/narf_descriptor.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
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



class NARFFeatures
{
    public:
		pcl::PointCloud<pcl::Narf36>::Ptr previous_Narf_features;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_keypoint_cloud ;
		pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr result;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_rgb;
		Eigen::Matrix4f globaltransform; 


        NARFFeatures()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&NARFFeatures::processPointCloud,this);
			previous_Narf_features = pcl::PointCloud<pcl::Narf36>::Ptr (new pcl::PointCloud<pcl::Narf36>);
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

		// creating point clouds......
		pcl::PCLPointCloud2 pcl_pc2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud_RGB);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

		std::vector<int> keypoint_indices2;
		keypoint_indices2.resize (keypoints.size ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoint_cloud (new pcl::PointCloud<pcl::PointXYZ>);// remove Nan and see....

		pcl::PointXYZ point;
		for(int i =0; i< keypoints.size();i++){
			double xc =keypoints[i].pt.x;
			double yc =keypoints[i].pt.y;

			if(isnan(cloud->at(xc,yc).x)){
				std::cout << "yo im nan" <<std::endl; 
				continue;
			}else{
				
				point.x = cloud->at(xc,yc).x;
				point.y = cloud->at(xc,yc).y;
				point.z = cloud->at(xc,yc).z;
				keypoint_cloud->push_back(point);
				keypoint_indices2[i] = 640*yc+xc;
			}

		}

        // Keypoints captured//
        // --------------------------------------------------------------------------------------------//
        // Starting feature extraction

        // Eigen::Affine3f scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		
		// Parameters for narf computation
		 float support_size = 0.1f;
		// float maxAngleWidth = (float) (57.0f * (M_PI / 180.0f));
		// float maxAngleHeight = (float) (43.0f * (M_PI / 180.0f));
		// float angularResolution = (float)(57.0f / 640.0f * (M_PI/180.0f));

		Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;

		int imageSizeX = 640;
		int imageSizeY = 480;
		// Center of projection. here, we choose the middle of the image.
		float centerX = 640.0f / 2.0f;
		float centerY = 480.0f / 2.0f;
		// Focal length. The value seen here has been taken from the original depth images.
		// It is safe to use the same value vertically and horizontally.
		float focalLengthX = 525.0f, focalLengthY = focalLengthX;
        // -----------------------------------------------
		// -----Create RangeImage from the PointCloud-----
		// -----------------------------------------------
		float noise_level = 0.00;
		float min_range = 0;
		int border_size = 0;
		// boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
		// pcl::RangeImage& range_image = *range_image_ptr;   
		// range_image.createFromPointCloud (*cloud, angularResolution,maxAngleWidth,maxAngleHeight,scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

		pcl::RangeImagePlanar rangeImagePlanar;
		rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
		centerX, centerY, focalLengthX, focalLengthX,
		sensorPose, pcl::RangeImage::CAMERA_FRAME,
		noise_level, min_range);
		//range_image.integrateFarRanges (far_ranges);
		std::cout << "Range image size :" << rangeImagePlanar.height <<" x "<< rangeImagePlanar.width <<std::endl;
		rangeImagePlanar.setUnseenToMaxRange();

		// pcl::visualization::PCLVisualizer viewer ("3D Viewer");
		// viewer.setBackgroundColor (1, 1, 1);
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
		// viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
		// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		// //viewer.addCoordinateSystem (1.0f);
		// //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
		// //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
		// viewer.initCameraParameters ();
		// setViewerPose (viewer, rangeImagePlanar.getTransformationToWorldSystem ());

		//Show range image ///
		pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
		range_image_widget.showRangeImage (rangeImagePlanar);

        // calculating features....

        // ------------------------------------------------------
        // -----Extract NARF descriptors for interest points-----
        // ------------------------------------------------------
        bool rotation_invariant = true;
		std::cout << "Range image size :" << rangeImagePlanar.height <<" x "<< rangeImagePlanar.width <<std::endl;
        pcl::NarfDescriptor narf_descriptor (&rangeImagePlanar, &keypoint_indices2);
        narf_descriptor.getParameters ().support_size = support_size;
        narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
        pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors(new pcl::PointCloud<pcl::Narf36>);
        narf_descriptor.compute (*narf_descriptors);
        std::cout << "Extracted "<<narf_descriptors->size ()<<" descriptors for "<<keypoints.size ()<< " keypoints.\n";

		double end = std::clock();
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;

		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");

		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoint_cloud, 0, 255, 0);
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud_RGB, 255, 255, 0);
		// viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		// viewer.addPointCloud(cloud_RGB, "cloud");
		// //viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud,cloud_with_normals, 25, 0.15,"normals"); 
		//viewer.addPointCloud(keypoint_cloud, keypoints_color_handler, "keypoints");

		// pcl::visualization::PCLVisualizer ICPView("ICP Viewer");


		std::cout << previous_Narf_features->size() << std::endl;

		if(previous_Narf_features->size()> 0 && previous_keypoint_cloud->size()>0){

			std::cout << "hola" << std::endl;

			pcl::registration::CorrespondenceEstimation<pcl::Narf36, pcl::Narf36> est;
    		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    		est.setInputSource(narf_descriptors);
    		est.setInputTarget(previous_Narf_features);
    		est.determineCorrespondences(*correspondences);

			// std::cout<<correspondences->size()<<std::endl;
			// for(int j = 0; j < correspondences->size(); j++){
			// 	std::cout<<correspondences->at(j)<<std::endl; 
			// }

			pcl::CorrespondencesPtr correspondences_result_reject_bad(new pcl::Correspondences());
			for(int x =0 ; x < correspondences->size(); x++){
				if(!correspondences->at(x).distance == 0){
					correspondences_result_reject_bad->push_back(correspondences->at(x));
				}
			}

			pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
			pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
			corr_rej_one_to_one.setInputCorrespondences(correspondences_result_reject_bad);
			corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);

			// std::cout<<"**********************************************"<<std::endl;
			// std::cout<<correspondences_result_rej_one_to_one->size()<<std::endl;
			// for(int j = 0; j < correspondences_result_rej_one_to_one->size(); j++){
			// 	std::cout<<correspondences_result_rej_one_to_one->at(j)<<std::endl; 
			// }

		// 	// Correspondance rejection RANSAC

			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
			pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
			rejector_sac.setInputSource(keypoint_cloud);
			rejector_sac.setInputTarget(previous_keypoint_cloud);
			rejector_sac.setInlierThreshold(0.05); // distance in m, not the squared distance
			// rejector_sac.setMaximumIterations(1000);
			// rejector_sac.setRefineModel(false);
			// rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);
			// rejector_sac.getCorrespondences(*correspondences_filtered);
			// correspondences.swap(correspondences_filtered);
			rejector_sac.getRemainingCorrespondences(*correspondences_result_rej_one_to_one,*correspondences_filtered);
			std::cout<<"**********************************************"<<std::endl;
			std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
			transform = rejector_sac.getBestTransformation(); // Transformation Estimation method 1

			for(int j = 0; j < correspondences_filtered->size(); j++){
				std::cout<<correspondences_filtered->at(j)<<std::endl; 
			}

		// 	// Transformation Estimation method 2
		// 	//pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
		// 	//transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);
			std::cout << "Estimated Transform:" << std::endl<< transform << std::endl;

			globaltransform = globaltransform * transform;

			std::cout << "Global Transform:" << std::endl<< globaltransform << std::endl;

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

			previous_Narf_features->clear();
			previous_keypoint_cloud->clear();
			pcl::copyPointCloud(*narf_descriptors,*previous_Narf_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);

			result->operator+=(*transformed_source);
			result_rgb->operator+=(*transformed_source_RGB);
			std::cout<<"cloud size :"<<cloud->width<<"x"<<cloud->height<<std::endl;
			std::cout<<"Concatanated point cloud size :"<<result->width<<"x"<<result->height<<std::endl;
			// ICPView.setBackgroundColor(0, 0, 0);
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb_cloud(result_rgb, 255, 255, 0);
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
			pcl::copyPointCloud(*narf_descriptors,*previous_Narf_features);
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

    void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
	{
		Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
		Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
		Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
		viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
				    look_at_vector[0], look_at_vector[1], look_at_vector[2],
				    up_vector[0], up_vector[1], up_vector[2]);
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
    NARFFeatures PC_obj;
    //ros::spin();
    ros::Rate r(0.01);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}