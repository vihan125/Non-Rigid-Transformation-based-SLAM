#include <iostream>
#include "ros/ros.h"

#include <boost/thread/thread.hpp>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>

#include <pcl/console/parse.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointField.h"
#include <bits/stdc++.h>

typedef pcl::PointXYZ PointType;

class Find_NARF_keypoints
{
    public:
        Find_NARF_keypoints()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&Find_NARF_keypoints::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
		
		
		pcl::PCLPointCloud2 pcl_pc2;
		//pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
		pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, point_cloud);
		//pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
		//Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
		
		//scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                //                                     point_cloud.sensor_origin_[1],
                //                                     point_cloud.sensor_origin_[2])) *
                //Eigen::Affine3f (point_cloud.sensor_orientation_);

		Eigen::Affine3f scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		
		// Parameters for narf computation
		//float angular_resolution = 0.5f;
		float support_size = 0.2f;
		float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
		float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
		float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;
		  
		double start = std::clock();  
		// -----------------------------------------------
		// -----Create RangeImage from the PointCloud-----
		// -----------------------------------------------
		float noise_level = 0.00;
		float min_range = 0.0f;
		int border_size = 1;
		boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;   
		range_image.createFromPointCloud (point_cloud, angularResolution,maxAngleWidth,maxAngleHeight,scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		//range_image.integrateFarRanges (far_ranges);
		range_image.setUnseenToMaxRange();

		pcl::visualization::PCLVisualizer viewer ("3D Viewer");
		viewer.setBackgroundColor (1, 1, 1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
		viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		//viewer.addCoordinateSystem (1.0f);
		//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
		//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
		viewer.initCameraParameters ();
		setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

		//Show range image ///
		pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
		range_image_widget.showRangeImage (range_image);

		// --------------------------------
		// -----Extract NARF keypoints-----
		// --------------------------------
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
 		//narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = support_size;
		narf_keypoint_detector.setKSearch(10);
		
		double end = std::clock();
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;

		pcl::PointCloud<int> keypoint_indices;
		//narf_keypoint_detector.compute (keypoint_indices); -> issue with PCL 1.10
		std::cout << "Here" << time_taken <<std::endl;
		std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

		// -------------------------------------
		// -----Show keypoints in 3D viewer-----
		// -------------------------------------
		// pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
		// keypoints.points.resize (keypoint_indices.points.size ());
		// for (size_t i=0; i<keypoint_indices.points.size (); ++i)
		// keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
		// viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
		// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		// while(!viewer.wasStopped())
		// {
		// 	viewer.spinOnce();
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
    Find_NARF_keypoints PC_obj;
    ros::Rate r(0.25);
    while (ros::ok())
    {
     ros::spinOnce();
     r.sleep();     
     }

    return 0;
}



