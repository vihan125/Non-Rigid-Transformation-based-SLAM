#include "tf_transform.hpp"

tf2_ros::Buffer* buffer = nullptr;
tf2_ros::TransformListener* listener = nullptr;

RegisterTF::RegisterTF(){

	//Assign subscriber
	this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&RegisterTF::processPointCloud,this);
	this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 10);
    this->publisher2 = this->nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/graph",10);

	// global clouds
	result_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    graph = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    graph->is_dense = false;
    result_rgb->is_dense = false;
	globaltransform = Eigen::Matrix4f::Identity();

}

void RegisterTF::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    int time = cloud_msg->header.stamp.sec;

    //Image
    sensor_msgs::Image image_;
    
    //Converting ROS point cloud to Image
    pcl::toROSMsg (*cloud_msg,image_);
    
    cv::Mat img,filtered_img,keypoints_mat, sharp_mat, contrast_mat, combi_mat;
    cv_bridge::CvImagePtr cvPtr;
    cvPtr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
    cvPtr->image.copyTo(img);

    cv::bilateralFilter(img,filtered_img,15,50,80,cv::BORDER_DEFAULT);
    std::vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(20);
    detector->detect(filtered_img, keypoints);

    //Show image
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

        if(isnan(cloud->at(xc,yc).x)){
            continue;
        }else{
            
            point.x = cloud->at(xc,yc).x;
            point.y = cloud->at(xc,yc).y;
            point.z = cloud->at(xc,yc).z;
            keypoint_cloud->push_back(point);
        }
    }

    //Remove Nan points from cloud
    cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    keypoint_cloud->is_dense = false;
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*keypoint_cloud,*keypoint_cloud, indices2);
    std::cout<<"Remove nan points from keypoints: "<< keypoint_cloud->points.size()<<std::endl;   

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

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    
    if(keypoint_cloud->points.size()>0){
        //Calculate FPFH features
      	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
		// Provide the original point cloud (without normals)
		fpfh_estimation.setInputCloud (keypoint_cloud);
		// Provide the point cloud with normals with no nan
		fpfh_estimation.setInputNormals (cloud_with_normals);
		fpfh_estimation.setSearchSurface(cloud);
		fpfh_estimation.setSearchMethod (tree);

		// Compute features
		fpfh_estimation.setKSearch(5);
		fpfh_estimation.compute (*fpfh_features);

        // descriptor keypoints size
        std::cout << "Calculated Features size: " << fpfh_features->points.size()<<"Calculated Keypointss size: " <<keypoint_cloud->points.size()<<std::endl;
    }
    
    // RGB cloud can hold Nan values 
    cloud_RGB->is_dense = false;

    // store tranform
    tf::StampedTransform transform;
    
    if(result_rgb->size()>0){
        
        std::string frame = cloud_msg->header.frame_id;
        ros::Time time = cloud_msg->header.stamp;

        sensor_msgs::PointCloud2 newCloud;
        sensor_msgs::PointCloud2 keyPointCloud;
        sensor_msgs::PointCloud2 Transformed_keyPointCloud;
        geometry_msgs::TransformStamped globalTransform;
        // globalTransform = buffer.lookupTransform("odom",frame,time);

        try{
            globalTransform = buffer->lookupTransform("odom",frame,time);
            tf_recived = true;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            tf_recived = false;
        }

        if(tf_recived){

            tf2::doTransform(*cloud_msg,newCloud,globalTransform);

            pcl::toROSMsg(*keypoint_cloud,keyPointCloud);

            tf2::doTransform(keyPointCloud,Transformed_keyPointCloud,globalTransform);

            pcl::PCLPointCloud2 tcloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB_T (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl_conversions::toPCL(newCloud, tcloud);
            pcl::fromPCLPointCloud2(tcloud, *cloud_RGB_T);

            pcl::PCLPointCloud2 kcloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_keypoints (new pcl::PointCloud<pcl::PointXYZ>);
            pcl_conversions::toPCL(Transformed_keyPointCloud, kcloud);
            pcl::fromPCLPointCloud2(kcloud, *transformed_keypoints);

            // // Deformationn graph
            for(int i = 0 ; i < fpfh_features->size(); i++){

                // Pushback the coordinate to relevnat descriptor entry
                Coordinate des_location;
                des_location.x = transformed_keypoints->at(i).x;
                des_location.y = transformed_keypoints->at(i).y;
                des_location.z = transformed_keypoints->at(i).z;

                DeformG[*fpfh_features->at(i).histogram].push_back(des_location);

                // For visualization purpose of the deformation graph
                pcl::PointXYZ des_point;
                des_point.x = des_location.x;
                des_point.y = des_location.y;
                des_point.z = des_location.z;

                graph->push_back(des_point);
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud (cloud_RGB_T);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*temp);

            result_rgb->operator+=(*temp);
            
            std::cout<<"map size :"<<result_rgb->size()<<std::endl;
            publisher.publish(result_rgb);
            publisher2.publish(graph);
        }

    }
    
    else{ // First cloud

        std::string frame = cloud_msg->header.frame_id;
        ros::Time time = cloud_msg->header.stamp;
        time.sec = time.sec+1;

        sensor_msgs::PointCloud2 newCloud;
        sensor_msgs::PointCloud2 keyPointCloud;
        sensor_msgs::PointCloud2 Transformed_keyPointCloud;
        geometry_msgs::TransformStamped globalTransform;
        // globalTransform = buffer.lookupTransform("odom",frame,time);

        try{
            globalTransform = buffer->lookupTransform("odom",frame,time);
            tf_recived = true;
        
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            tf_recived = false;
        }
        
        if(tf_recived){

            tf2::doTransform(*cloud_msg,newCloud,globalTransform);

            pcl::toROSMsg(*keypoint_cloud,keyPointCloud);

            tf2::doTransform(keyPointCloud,Transformed_keyPointCloud,globalTransform);

            pcl::PCLPointCloud2 tcloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB_T (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl_conversions::toPCL(newCloud, tcloud);
            pcl::fromPCLPointCloud2(tcloud, *cloud_RGB_T);
            
            pcl::PCLPointCloud2 kcloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_keypoints (new pcl::PointCloud<pcl::PointXYZ>);
            pcl_conversions::toPCL(Transformed_keyPointCloud, kcloud);
            pcl::fromPCLPointCloud2(kcloud, *transformed_keypoints);

            // // Deformationn graph first entry
            for(int i = 0 ; i < fpfh_features->size(); i++){

                // Pushback the coordinate to relevnat descriptor entry
                Coordinate des_location;
                des_location.x = transformed_keypoints->at(i).x;
                des_location.y = transformed_keypoints->at(i).y;
                des_location.z = transformed_keypoints->at(i).z;

                DeformG[*fpfh_features->at(i).histogram].push_back(des_location);

                // For visualization purpose of the deformation graph
                pcl::PointXYZ des_point;
                des_point.x = des_location.x;
                des_point.y = des_location.y;
                des_point.z = des_location.z;

                graph->push_back(des_point);

            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud (cloud_RGB_T);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*temp);

            pcl::copyPointCloud(*temp,*result_rgb);
            
            std::cout<<"len :"<<temp->size()<<std::endl;
            publisher.publish(result_rgb);
            // publisher.publish(graph);
        }
    }
    
}

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"Registration_TF");
    std::cout << "Register point clouds from odometry data" << std::endl;

    // Intializing Tflistner and buffer
    buffer = new tf2_ros::Buffer(ros::Duration(60));
    listener = new tf2_ros::TransformListener(*buffer);

    // handle ROS communication events
    RegisterTF TF;
    //ros::spin();
    ros::Rate r(2);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;

}