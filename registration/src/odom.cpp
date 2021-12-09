#include "odom.hpp"

RegisterOdom::RegisterOdom()
{
	//Assign subscriber
	this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&RegisterOdom::processPointCloud,this);
	this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 10);
    this->publisher2 = this->nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("/graph",10);
	this->client = this->nh.serviceClient<registration::getTransform>("get_Transform");

	// global clouds
	result_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    graph = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    graph->is_dense = false;
    result_rgb->is_dense = false;
	globaltransform = Eigen::Matrix4f::Identity();

}

void RegisterOdom::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

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

    srv.request.date = time;
    
    if(result_rgb->size()>0){

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


                std::cout<<"Global T :"<<std::endl;
                std::cout<<globaltransform<<std::endl;
                
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

                pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);
                pcl::transformPointCloud(*keypoint_cloud,*transformed_keypoints,globaltransform);

                // Deformationn graph
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
                sor.setInputCloud (transformed_source_RGB);
                sor.setLeafSize (0.05f, 0.05f, 0.05f);
                sor.filter (*temp);

                result_rgb->operator+=(*temp);
                
                std::cout<<"map size :"<<result_rgb->size()<<std::endl;
                publisher.publish(result_rgb);
                publisher2.publish(graph);

            }else{

                std::cout<<"unable to find a transformation ... Need to handle exception !!"<<std::endl;
                	
            }
    }
    else{ // First cloud

        if(client.call(srv)){

            std::cout<<"First time"<<std::endl;
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


            std::cout<<"Global T :"<<std::endl;
            std::cout<<globaltransform<<std::endl;
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);
            pcl::transformPointCloud(*keypoint_cloud,*transformed_keypoints,globaltransform);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

            // Deformationn graph first entry
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

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud (transformed_source_RGB);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*temp);

            pcl::copyPointCloud(*temp,*result_rgb);
            
            std::cout<<"len :"<<temp->size()<<std::endl;
            publisher.publish(result_rgb);
            publisher.publish(graph);
        }

    }
}