#include "reg_3dc.hpp"



DSCFeatures::DSCFeatures()
{
	//Assign subscriber
	this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&DSCFeatures::processPointCloud,this);
	this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 1);
	this->client = this->nh.serviceClient<registration::getTransform>("get_Transform");

	// global clouds
	previous_dc_descriptors = pcl::PointCloud<pcl::ShapeContext1980>::Ptr (new pcl::PointCloud<pcl::ShapeContext1980>);
	previous_keypoint_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	result = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	result_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	globaltransform = Eigen::Matrix4f::Identity();

}

void DSCFeatures::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	double start = std::clock();

	int time = cloud_msg->header.stamp.sec;
	
	//Image
	sensor_msgs::Image image_;
	//Converting ROS point cloud to Image
	pcl::toROSMsg (*cloud_msg,image_);
	//sensor_msgs::ImageConstPtr img_msg = &image_;
	cv::Mat img,filtered_img,keypoints_mat, sharp_mat, contrast_mat, combi_mat;
	cv_bridge::CvImagePtr cvPtr;
	cvPtr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
	cvPtr->image.copyTo(img);
	

	cv::bilateralFilter(img,filtered_img,5,5,5,cv::BORDER_DEFAULT);
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::ORB> detector = cv::ORB::create(500);
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
	
	// if(keypoints.size()<10){
	// 	pcl::PointXYZ point;
	// 	for(int i =0; i< keypoints.size();i++){
	// 		double xc =keypoints[i].pt.x;
	// 		double yc =keypoints[i].pt.y;

	// 		if(isnan(cloud->at(xc,yc).x)){
	// 			continue;
	// 		}else{
				
	// 			point.x = cloud->at(xc,yc).x;
	// 			point.y = cloud->at(xc,yc).y;
	// 			point.z = cloud->at(xc,yc).z;
	// 			keypoint_cloud->push_back(point);

	// 			if(xc<640 && yc<480){

	// 				point.x = cloud->at(xc+1,yc).x;
	// 				point.y = cloud->at(xc+1,yc).y;
	// 				point.z = cloud->at(xc+1,yc).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc-1,yc).x;
	// 				point.y = cloud->at(xc-1,yc).y;
	// 				point.z = cloud->at(xc-1,yc).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc-1,yc+1).x;
	// 				point.y = cloud->at(xc-1,yc+1).y;
	// 				point.z = cloud->at(xc-1,yc+1).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc+1,yc+1).x;
	// 				point.y = cloud->at(xc+1,yc+1).y;
	// 				point.z = cloud->at(xc+1,yc+1).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc-1,yc-1).x;
	// 				point.y = cloud->at(xc-1,yc-1).y;
	// 				point.z = cloud->at(xc-1,yc-1).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc+1,yc-1).x;
	// 				point.y = cloud->at(xc+1,yc-1).y;
	// 				point.z = cloud->at(xc+1,yc-1).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc,yc+1).x;
	// 				point.y = cloud->at(xc,yc+1).y;
	// 				point.z = cloud->at(xc,yc+1).z;
	// 				keypoint_cloud->push_back(point);

	// 				point.x = cloud->at(xc,yc-1).x;
	// 				point.y = cloud->at(xc,yc-1).y;
	// 				point.z = cloud->at(xc,yc-1).z;
	// 				keypoint_cloud->push_back(point);

	// 			}
	// 		}

	// 	}	

	// }else{
	// 	pcl::PointXYZ point;
	// 	for(int i =0; i< keypoints.size();i++){
	// 		double xc =keypoints[i].pt.x;
	// 		double yc =keypoints[i].pt.y;

	// 		if(isnan(cloud->at(xc,yc).x)){
	// 			continue;
	// 		}else{
				
	// 			point.x = cloud->at(xc,yc).x;
	// 			point.y = cloud->at(xc,yc).y;
	// 			point.z = cloud->at(xc,yc).z;
	// 			keypoint_cloud->push_back(point);
	// 		}

	// 	}
	// }

	pcl::PointXYZ point;
	for(int i =0; i< keypoints.size();i++){

		if(keypoint_cloud->size()>200){
				break;
		}
		else{
			double xc =keypoints[i].pt.x;
			double yc =keypoints[i].pt.y;

			if(isnan(cloud->at(xc,yc).x)){
				continue;
			}else{
				
				point.x = cloud->at(xc,yc).x;
				point.y = cloud->at(xc,yc).y;
				point.z = cloud->at(xc,yc).z;
				keypoint_cloud->push_back(point);
			}
		}

	}

	std::cout<<"Keypoint cloud size: "<< keypoint_cloud->points.size()<<std::endl;

	if(keypoint_cloud->points.size()>20){

		//Remove Nan points from cloud
		cloud->is_dense = false;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		std::cout << "Calculation of cloud done" << cloud->points.size()<<std::endl;

		//double start = std::clock(); 
		
		// Compute the normals
		pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		cloud_with_normals->is_dense = false;
		
		normal_estimation.setInputCloud (cloud);
		normal_estimation.setSearchMethod (tree);	
		//normal_estimation.setRadiusSearch(0.03);
		normal_estimation.setKSearch(5);

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

		std::cout << "Calculation of Normals done" << normals_Nan->points.size()<<std::endl;

		keypoint_cloud->is_dense = false;
		std::vector<int> indices2;
		pcl::removeNaNFromPointCloud(*keypoint_cloud,*keypoint_cloud, indices2);
		std::cout<<"Remove nan points from keypoints: "<< keypoint_cloud->points.size()<<std::endl;   

		// 3DSC estimation object.
		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sc3d;
		sc3d.setInputCloud(keypoint_cloud);
		sc3d.setInputNormals(normals_Nan);
		sc3d.setSearchSurface(cloud);
		sc3d.setSearchMethod(tree);
		// Search radius, to look for neighbors. It will also be the radius of the support sphere.
		sc3d.setRadiusSearch(0.05);
		// The minimal radius value for the search sphere, to avoid being too sensitive
		// in bins close to the center of the sphere.
		sc3d.setMinimalRadius(0.005);
		// Radius used to compute the local point density for the neighbors
		// (the density is the number of points within that radius).
		sc3d.setPointDensityRadius(0.01);
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr dc_descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
		sc3d.compute(*dc_descriptors);

		// Display and retrieve the shape context descriptor vector for the 0th point.
		std::cout << "Calculated Features size: " << dc_descriptors->points.size () <<std::endl;
		
		//pcl::FPFHSignature33 descriptor = (*fpfh_features)[0];
		//std::cout << descriptor << std::endl;
		double end = std::clock();
		double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		std::cout << "Time taken :" << time_taken <<std::endl;

		// pcl::visualization::PCLVisualizer viewer;

		if(previous_dc_descriptors->size()> 0 && previous_keypoint_cloud->size()>0){
			std::cout << "hola" << std::endl;

			double start_r = std::clock();
			// handle not enough keypoints..

			// 1. initial alignment of point clouds
			Eigen::Matrix4f initTransform = Eigen::Matrix4f::Identity();
			pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::ShapeContext1980> initSAC;
			pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
			initSAC.setMinSampleDistance(0.01);
			initSAC.setMaxCorrespondenceDistance(0.5);
			initSAC.setMaximumIterations(1500);

			initSAC.setInputCloud(keypoint_cloud);
			initSAC.setSourceFeatures(dc_descriptors);

			initSAC.setInputTarget(previous_keypoint_cloud);
			initSAC.setTargetFeatures(previous_dc_descriptors);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr initTransCloud (new pcl::PointCloud<pcl::PointXYZ>);
			initSAC.align(*initTransCloud);
			initTransform = initSAC.getFinalTransformation();

			double end_r = std::clock();
			double time_r = double(end_r-start_r) / double(CLOCKS_PER_SEC);
			std::cout << "done init :" <<time_r<<" seconds"<<std::endl;
			// 2. Fine alignment using ICP

			// 2 stage ICP pipeline
			Eigen::Matrix4f finalTransform = Eigen::Matrix4f::Identity();
			pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
			pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp2;

			icp.setMaxCorrespondenceDistance(0.01);
			icp.setRANSACOutlierRejectionThreshold(0.15);
			icp.setTransformationEpsilon(0.01);
			icp.setMaximumIterations(10000);
			icp.setInputCloud(initTransCloud);
			icp.setInputTarget(previous_keypoint_cloud);

			pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
			icp.align(*finalCloud);

			if(icp.hasConverged()){

				finalTransform = icp.getFinalTransformation() * initTransform;
			}
			else{
				std::cout << "ICP2.."<<std::endl;
				icp2.setMaxCorrespondenceDistance(0.05);
				icp2.setRANSACOutlierRejectionThreshold(0.2);
				icp2.setTransformationEpsilon(0.1);
				icp2.setMaximumIterations(50000);
				icp2.setInputCloud(initTransCloud);
				icp2.setInputTarget(previous_keypoint_cloud);

				pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
				icp2.align(*finalCloud);	
				finalTransform = icp2.getFinalTransformation() * initTransform;	
			}

			double end2_r = std::clock();
			double time2_r = double(end2_r-end_r) / double(CLOCKS_PER_SEC);
			std::cout << "done ICP :" <<time2_r<<" seconds"<<std::endl;

			double time2_reg = double(end2_r-start_r) / double(CLOCKS_PER_SEC);
			std::cout << "done registration :" <<time2_reg<<" seconds"<<std::endl;
			//global transform --

			if(icp.hasConverged() || icp2.hasConverged()){

				srv.request.date = time;
				if(client.call(srv)){
				std::cout<<"service responded"<<std::endl;
				globaltransform = Eigen::Matrix4f::Identity();
				globaltransform(0,0) = srv.response.matrix.entry_0_0;
				globaltransform(0,1) = srv.response.matrix.entry_0_1;
				globaltransform(0,2) = srv.response.matrix.entry_0_2;
				globaltransform(0,3) = srv.response.matrix.entry_0_3;

				globaltransform(1,0) = srv.response.matrix.entry_1_0;
				globaltransform(1,1) = srv.response.matrix.entry_1_1;
				globaltransform(1,2) = srv.response.matrix.entry_1_2;
				globaltransform(1,3) = srv.response.matrix.entry_1_3;

				globaltransform(2,0) = srv.response.matrix.entry_2_0;
				globaltransform(2,1) = srv.response.matrix.entry_2_1;
				globaltransform(2,2) = srv.response.matrix.entry_2_2;
				globaltransform(2,3) = srv.response.matrix.entry_2_3;

				globaltransform(3,0) = srv.response.matrix.entry_3_0;
				globaltransform(3,1) = srv.response.matrix.entry_3_1;
				globaltransform(3,2) = srv.response.matrix.entry_3_2;
				globaltransform(3,3) = srv.response.matrix.entry_3_3;
			}
				// globaltransform = Transformation::getTransform(time);
				// globaltransform = globaltransform * finalTransform;
			else{
				srv.request.date = time;
				if(client.call(srv)){
				std::cout<<"service responded"<<std::endl;
				globaltransform = Eigen::Matrix4f::Identity();
				globaltransform(0,0) = srv.response.matrix.entry_0_0;
				globaltransform(0,1) = srv.response.matrix.entry_0_1;
				globaltransform(0,2) = srv.response.matrix.entry_0_2;
				globaltransform(0,3) = srv.response.matrix.entry_0_3;

				globaltransform(1,0) = srv.response.matrix.entry_1_0;
				globaltransform(1,1) = srv.response.matrix.entry_1_1;
				globaltransform(1,2) = srv.response.matrix.entry_1_2;
				globaltransform(1,3) = srv.response.matrix.entry_1_3;

				globaltransform(2,0) = srv.response.matrix.entry_2_0;
				globaltransform(2,1) = srv.response.matrix.entry_2_1;
				globaltransform(2,2) = srv.response.matrix.entry_2_2;
				globaltransform(2,3) = srv.response.matrix.entry_2_3;

				globaltransform(3,0) = srv.response.matrix.entry_3_0;
				globaltransform(3,1) = srv.response.matrix.entry_3_1;
				globaltransform(3,2) = srv.response.matrix.entry_3_2;
				globaltransform(3,3) = srv.response.matrix.entry_3_3;
			}
		}

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

			previous_dc_descriptors->clear();
			previous_keypoint_cloud->clear();
			pcl::copyPointCloud(*dc_descriptors,*previous_dc_descriptors);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::VoxelGrid<pcl::PointXYZRGB> sor;
			sor.setInputCloud (transformed_source_RGB);
			sor.setLeafSize (0.05f, 0.05f, 0.05f);
			sor.filter (*temp);

			result->operator+=(*transformed_source);
			result_rgb->operator+=(*temp);
			std::cout<<"cloud size :"<<cloud->width<<"x"<<cloud->height<<std::endl;
			std::cout<<"Concatanated point cloud size :"<<result->width<<"x"<<result->height<<std::endl;

			publisher.publish(result_rgb);

			// pcl::visualization::PCLVisualizer viewer;
			// while(!viewer.wasStopped())
			// {
			// 	viewer.spinOnce();
			// }
			
			
		}
		else{
			std::cout << "first_time" << std::endl;
			srv.request.date = time;
			if(client.call(srv)){
				std::cout<<"service responded"<<std::endl;
				globaltransform = Eigen::Matrix4f::Identity();
				globaltransform(0,0) = srv.response.matrix.entry_0_0;
				globaltransform(0,1) = srv.response.matrix.entry_0_1;
				globaltransform(0,2) = srv.response.matrix.entry_0_2;
				globaltransform(0,3) = srv.response.matrix.entry_0_3;

				globaltransform(1,0) = srv.response.matrix.entry_1_0;
				globaltransform(1,1) = srv.response.matrix.entry_1_1;
				globaltransform(1,2) = srv.response.matrix.entry_1_2;
				globaltransform(1,3) = srv.response.matrix.entry_1_3;

				globaltransform(2,0) = srv.response.matrix.entry_2_0;
				globaltransform(2,1) = srv.response.matrix.entry_2_1;
				globaltransform(2,2) = srv.response.matrix.entry_2_2;
				globaltransform(2,3) = srv.response.matrix.entry_2_3;

				globaltransform(3,0) = srv.response.matrix.entry_3_0;
				globaltransform(3,1) = srv.response.matrix.entry_3_1;
				globaltransform(3,2) = srv.response.matrix.entry_3_2;
				globaltransform(3,3) = srv.response.matrix.entry_3_3;
			}else{
				globaltransform = Eigen::Matrix4f::Identity();	
			}
			
			std::cout << "current transform :" << globaltransform <<std::endl;
			pcl::copyPointCloud(*dc_descriptors,*previous_dc_descriptors);
			pcl::copyPointCloud(*keypoint_cloud,*previous_keypoint_cloud);

			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::transformPointCloud(*cloud, *transformed_source, globaltransform);
			pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::VoxelGrid<pcl::PointXYZRGB> sor;
			sor.setInputCloud (transformed_source_RGB);
			sor.setLeafSize (0.05f, 0.05f, 0.05f);
			sor.filter (*temp);

			result->operator+=(*transformed_source);
			result_rgb->operator+=(*temp);
			
			publisher.publish(result_rgb);
			

			// pcl::visualization::PCLVisualizer viewer;
			// while(!viewer.wasStopped())
			// {
			// 	viewer.spinOnce();
			// }	

		}

		// while(!viewer.wasStopped())
		// {
		// 	viewer.spinOnce();
		// }


		std::cout<<"***********************************************************"<<std::endl;
	}
	else{
		previous_keypoint_cloud->clear();
		previous_dc_descriptors->clear();
	}

	}
	

}