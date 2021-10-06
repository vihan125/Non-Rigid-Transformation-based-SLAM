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
			this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 1);

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

		pcl::RangeImagePlanar rangeImagePlanar;
		rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
		centerX, centerY, focalLengthX, focalLengthX,
		sensorPose, pcl::RangeImage::CAMERA_FRAME,
		noise_level, min_range);
		//range_image.integrateFarRanges (far_ranges);
		std::cout << "Range image size :" << rangeImagePlanar.height <<" x "<< rangeImagePlanar.width <<std::endl;
		rangeImagePlanar.setUnseenToMaxRange();

		//Show range image ///
		pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
		range_image_widget.showRangeImage (rangeImagePlanar);

        // calculating features....

        // ------------------------------------------------------
        // -----Extract NARF descriptors for interest points-----
        // ------------------------------------------------------
        bool rotation_invariant = true;
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


		std::cout << previous_Narf_features->size() << std::endl;

		if(previous_Narf_features->size()> 0 && previous_keypoint_cloud->size()>0){

			std::cout << "hola" << std::endl;
            // correspondence rejection pipeline

			// 1. correspondence estimation using equalent features
			pcl::CorrespondencesPtr correspondences (new pcl::Correspondences());
			pcl::Correspondence c;

			for(int i = 0; i<narf_descriptors->size(); i++){

				for(int j = 0; j < previous_Narf_features->size(); j++){
                    std::cout <<i<<","<<j<<std::endl;
					if(*narf_descriptors->at(i).descriptor == *previous_Narf_features->at(j).descriptor){

						c.index_query = i;
						c.index_match = j;
                        pcl::PointXYZ point1;
                        point1.x = narf_descriptors->at(i).x;
                        point1.y = narf_descriptors->at(i).y;
                        point1.z = narf_descriptors->at(i).z;

                        pcl::PointXYZ point2;
                        point2.x = previous_Narf_features->at(i).x;
                        point2.y = previous_Narf_features->at(i).y;
                        point2.z = previous_Narf_features->at(i).z;

						c.distance = pcl::squaredEuclideanDistance(point1,point2);
						correspondences->push_back(c);
					}
				}
			}
			std::cout << "After feature matching :" << correspondences->size()<<std::endl;

			// 2. Reject correspondances which have dostance greater than median distance
			pcl::CorrespondencesPtr correspondences_distance (new pcl::Correspondences());
			pcl::registration::CorrespondenceRejectorMedianDistance distanceRejector;
			distanceRejector.setInputCorrespondences(correspondences);
			distanceRejector.setMedianFactor(0.16);
			distanceRejector.getCorrespondences(*correspondences_distance);
			std::cout << "After distant filtering :" << correspondences_distance->size()<<std::endl;

			// 3. Correspondance rejection RANSAC
			Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
			pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
			rejector_sac.setInputSource(keypoint_cloud);
			rejector_sac.setInputTarget(previous_keypoint_cloud);
			rejector_sac.setInlierThreshold(0.1); // distance in m, not the squared distance
			rejector_sac.setMaximumIterations(1000000);
			rejector_sac.setRefineModel(false);
			rejector_sac.setInputCorrespondences(correspondences_distance);
			rejector_sac.getCorrespondences(*correspondences_filtered);

            std::cout << "Final correspondences :" << correspondences_filtered->size()<<std::endl;

			for(int j = 0; j < correspondences_filtered->size(); j++){
				std::cout<<correspondences_filtered->at(j)<<std::endl; 
			};
            
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

			previous_Narf_features->clear();
			previous_keypoint_cloud->clear();
			pcl::copyPointCloud(*narf_descriptors,*previous_Narf_features);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);
			result->operator+=(*transformed_source);
			result_rgb->operator+=(*transformed_source_RGB);
			std::cout<<"cloud size :"<<cloud->width<<"x"<<cloud->height<<std::endl;
			std::cout<<"Concatanated point cloud size :"<<result->width<<"x"<<result->height<<std::endl;

			publisher.publish(result_rgb);


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
    ros::Rate r(0.05);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;
}