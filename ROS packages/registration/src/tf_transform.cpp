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

    static tf2_ros::TransformBroadcaster br;

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

    //testing purpose
    // std::vector<std::pair<double,double>> keypoint_places;
    
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
            // std::pair<double,double> p;
            // p.first = xc;
            // p.second = yc;
            // keypoint_places.push_back(p);
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
        std::cout << "Calculated Features size: " << fpfh_features->points.size()<<std::endl<<"Calculated Keypointss size: " <<keypoint_cloud->points.size()<<std::endl;
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
            ros::Duration(1.0).sleep();
            globalTransform = buffer->lookupTransform("odom",frame,time);
            tf_recived = true;
        }
        catch (tf2::TransformException &ex) {
            // ROS_WARN("%s",ex.what());
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

            std::vector<Coordinate> Existing_matches;
            std::vector<Coordinate> New_frame_matches;

            bool loop_closed = false;
            int not_the_same = 0;

            double exist_x_sum = 0;
            double exist_y_sum = 0;
            double exist_z_sum = 0;

            double new_x_sum = 0;
            double new_y_sum = 0;
            double new_z_sum = 0;

            std::vector <Coordinate> erronous_cod; // ---> Load to GPU
            std::vector <Coordinate> correct_cod;
            std::vector <cv::Mat> Unique_T; // ---> Load to GPU

            // // Deformationn graph
            for(int i = 0 ; i < fpfh_features->size(); i++){

                // Pushback the coordinate to relevnat descriptor entry
                Coordinate des_location;
                des_location.x = transformed_keypoints->at(i).x;
                des_location.y = transformed_keypoints->at(i).y;
                des_location.z = transformed_keypoints->at(i).z;

                float hist = *fpfh_features->at(i).histogram;

                if(DeformG.find(hist) != DeformG.end()){

                    std::cout<<"****************** Existing Feature *****************"<<std::endl;
                    // Feature encountered before
                    std::vector<Coordinate> Locations = DeformG[hist];

                    float d0 = 10;
                    int selected_pos = 0;
                    // Check if it is observed in the same location before
                    for (int i = 0 ; i < Locations.size(); i++){
                        
                        // std::cout<<"old coordinates -"<<Locations[i].x<<","<<Locations[i].y<<","<<Locations[i].z<<std::endl;
                        // std::cout<<"New coordinates -"<<des_location.x<<","<<des_location.y<<","<<des_location.z<<std::endl;

                        float d = Locations[i].getDistance(des_location);
                        // std::cout<<d<<std::endl;

                        if (d<d0){
                            d0 = d;
                            selected_pos = i;
                        }

                    }

                    if( 0.000001 <= d0 <= 0.0001) {
                    
                        std::cout<<"----++ Found a candidate ++----"<<std::endl;

                        // Puting coordinates to the matched lists
                        Existing_matches.push_back(Locations[selected_pos]);
                        New_frame_matches.push_back(des_location);
                        std::cout<<"old coordinates -"<<Locations[selected_pos].x<<","<<Locations[selected_pos].y<<","<<Locations[selected_pos].z<<std::endl;
                        std::cout<<"New coordinates -"<<des_location.x<<","<<des_location.y<<","<<des_location.z<<std::endl;
                        
                        // Getting the sum of matched exisying x,yz values to calculate centroid
                        exist_x_sum = exist_x_sum + Locations[selected_pos].x;
                        exist_y_sum = exist_y_sum + Locations[selected_pos].y;
                        exist_z_sum = exist_z_sum + Locations[selected_pos].z;

                        // Getting the sum of matched new x,yz values to calculate centroid
                        new_x_sum = new_x_sum + des_location.x;
                        new_y_sum = new_y_sum + des_location.y;
                        new_z_sum = new_z_sum + des_location.z;

                        // Differences
                        double x_dif = Locations[selected_pos].x - des_location.x;
                        double y_dif = Locations[selected_pos].y - des_location.y;
                        double z_dif = Locations[selected_pos].z - des_location.z;
                        
                        // get How many points are there with a actual difference 
                        if(x_dif > 0 || y_dif > 0 || z_dif >0){

                            not_the_same = not_the_same + 1;
                            erronous_cod.push_back(des_location);
                            correct_cod.push_back(Locations[selected_pos]);

                        }

                        double angle_zy = atan((Locations[selected_pos].y - des_location.y) / (Locations[selected_pos].z - des_location.z) );
                        double angle_zx = atan((Locations[selected_pos].x - des_location.x) / (Locations[selected_pos].z - des_location.z) );
                        double angle_xy = atan((Locations[selected_pos].y - des_location.y) / (Locations[selected_pos].x - des_location.x) );
                        std::cout<<"zy angle :"<< angle_zy << std::endl;
                        std::cout<<"zx angle :"<< angle_zx << std::endl;
                        std::cout<<"xy angle :"<< angle_xy << std::endl;

                    }
                    else{
                                                            
                        // Feature was not observed in the encountered location before
                        DeformG[hist].push_back(des_location);
                    
                    }

                    // Check weather a loop was closed

                }
                else{
                    std::cout<<"++++++++++++++ New Feature +++++++++++++++++"<<std::endl;
                    // Feature encountered for the first time
                    DeformG[hist].push_back(des_location);
                }
    

                // For visualization purpose of the deformation graph
                pcl::PointXYZ des_point;
                des_point.x = des_location.x;
                des_point.y = des_location.y;
                des_point.z = des_location.z;

                graph->push_back(des_point);
            }

            // check weather matched features points are greater than 3
            if(Existing_matches.size()>=3 && not_the_same > 0){

                //Can calculate the transform and close a local loop
                std::cout<<"++++++++++++++++ Loop closure possible ++++++++++++++++"<<std::endl;

                // Create matrixes
                cv::Mat ExiM (3,Existing_matches.size(),CV_64FC1);
                cv::Mat NewF (3,New_frame_matches.size(),CV_64FC1);

                // Calculate centroids
                int matched_points = Existing_matches.size();

                double exist_x_centroid = exist_x_sum / matched_points;
                double exist_y_centroid = exist_y_sum / matched_points;
                double exist_z_centroid = exist_z_sum / matched_points;

                std::vector <double> exist_cen {exist_x_centroid, exist_y_centroid, exist_z_centroid};

                double new_x_centroid = new_x_sum / matched_points;
                double new_y_centroid = new_y_sum / matched_points;
                double new_z_centroid = new_z_sum / matched_points;

                std::vector <double> new_cen {new_x_centroid, new_y_centroid, new_z_centroid};


                for(int i = 0; i < Existing_matches.size();i++){

                    ExiM.at<double>(0,i) = Existing_matches[i].x - exist_x_centroid;
                    ExiM.at<double>(1,i) = Existing_matches[i].y - exist_y_centroid;
                    ExiM.at<double>(2,i) = Existing_matches[i].z - exist_z_centroid;

                    NewF.at<double>(0,i) = New_frame_matches[i].x - new_x_centroid;
                    NewF.at<double>(1,i) = New_frame_matches[i].y - new_y_centroid;
                    NewF.at<double>(2,i) = New_frame_matches[i].z - new_z_centroid;

                }

                std::cout<<"Matrixes ..\n"<<std::endl;

                std::cout<<"New (A) .."<<std::endl;
                std::cout<< NewF <<std::endl;
                std::cout<<"\n"<<std::endl;

                std::cout<<"Existing (B) .."<<std::endl;
                std::cout<< ExiM <<std::endl;

                cv::Mat ExiM_T;
                cv::transpose(ExiM,ExiM_T);

                std::cout<<"Transpose (B) .."<<std::endl;
                std::cout<< ExiM_T <<std::endl;

                // Calculating H matrix
                cv::Mat H = NewF * ExiM_T;

                std::cout<<"H .."<<std::endl;
                std::cout<< H <<std::endl;

                // Output of Single value decomposition
                cv::Mat S;
                cv::Mat U;
                cv::Mat Vt;

                // Calculate SVD
                cv::SVD::compute(H, S, U, Vt);

                // Transpose of Vt
                cv::Mat V;
                cv::transpose(Vt, V);

                // Transpose of U
                cv::Mat Ut;
                cv::transpose(U, Ut);

                // Rotation Matrix
                cv::Mat R = V * Ut; // ----> Rotation matrix to GPU
                std::cout<<"R .."<<std::endl;
                std::cout<< R <<std::endl;

                // special Reflection case handling
                double det = cv::determinant(R);

                if(det < 0){
    
                     V.col(2) = V.col(2) * -1 ;
                     R = V * Ut;
                }
                
                // Centroid matrixes
                cv::Mat Exist_Cen (3,1,CV_64FC1,&exist_cen.front());
                cv::Mat New_Cen (3,1,CV_64FC1,&new_cen.front());

                // calculation of translation using centroids
                cv::Mat t = Exist_Cen - (R * New_Cen ); // -----> common translation to GPU

                // Calculating individual translation component

                cv::Mat u_err (3, 1, CV_64FC1);
                cv::Mat corr (3, 1, CV_64FC1);

                for(int i =0 ; i < erronous_cod.size() ; i++){

                    u_err.at<double>(0,0) = erronous_cod[i].x;
                    u_err.at<double>(1,0) = erronous_cod[i].y;
                    u_err.at<double>(2,0) = erronous_cod[i].z;

                    cv::Mat ajusted_cord;

                    ajusted_cord = ( R * u_err ) + t;

                    cv::Mat indi_t;

                    corr.at<double>(0,0) = correct_cod[i].x;
                    corr.at<double>(1,0) = correct_cod[i].y;
                    corr.at<double>(2,0) = correct_cod[i].z;

                    indi_t = corr - ajusted_cord;

                    Unique_T.push_back(indi_t);
                }

                // Radius calculation
                float r = 0.004444445;

                for(int i = 0; i < (New_frame_matches.size()-1); i++){

                    for(int j = i+1; j < New_frame_matches.size(); j++){

                        float r_p = New_frame_matches[i].getDistance(New_frame_matches[j]);

                        if(r_p < r){
                            r = 0.5625 * r_p; // (9/16 = 0.5625)
                        }

                    }
                }

                // checking key points points before deformation
                // for(int i =0; i < keypoint_places.size(); i++){
                //     std::pair<double,double> poi = keypoint_places[i];
                //     std::cout<<cloud_RGB_T->at(poi.first,poi.second).x<<","<<cloud_RGB_T->at(poi.first,poi.second).y<<","<<cloud_RGB_T->at(poi.first,poi.second).z<<std::endl;
                // }

                // -----------------  CUDA part --------------------------

                // Uploading point cloud to the device
                pcl::gpu::DeviceArray<pcl::PointXYZRGB> cloud_device;

                cloud_device.upload(cloud_RGB_T->points); /// upload point cloud to gpu

                // arrrays to hold rotation, translation, erronous points, unique translation and radius
                double *rot = (double*)malloc(9*sizeof(double));
                double *trans = (double*)malloc(3*sizeof(double));
                double *err_ps = (double*)malloc(3*erronous_cod.size()*sizeof(double));
                double *trans_uniq = (double*)malloc(3*erronous_cod.size()*sizeof(double));
                float *radius = &r;
                int err_p_size = erronous_cod.size();

                // populating rotation matrix array (row major)
                for(int i =0; i<3; i++){

                    rot[(3*i)+0] = R.at<double>(i,0);
                    rot[(3*i)+1] = R.at<double>(i,1);
                    rot[(3*i)+2] = R.at<double>(i,2);

                }

                // populating translation matrix array [x,y,z]
                trans[0] = t.at<double>(0,0);
                trans[1] = t.at<double>(1,0);
                trans[2] = t.at<double>(2,0);

                // populating erronus points ([x1,y1,z1,x2,y2,z2,...xn,,yn,zn])
                for(int i = 0; i < erronous_cod.size(); i++){

                    err_ps[(3*i)+0] = erronous_cod[i].x;
                    err_ps[(3*i)+1] = erronous_cod[i].y;
                    err_ps[(3*i)+2] = erronous_cod[i].z;

                }

                // populating unnique translations of erronus points ([x1,y1,z1,x2,y2,z2,...xn,,yn,zn])
                for(int i = 0; i < erronous_cod.size(); i++){

                    trans_uniq[(3*i)+0] = Unique_T[i].at<double>(0,0);
                    trans_uniq[(3*i)+1] = Unique_T[i].at<double>(1,0);
                    trans_uniq[(3*i)+2] = Unique_T[i].at<double>(2,0);

                }
                
                std::cout<<"GPU task started ....."<<std::endl;
                double start = std::clock();
                // Calling cuda kernal though interface
                deformCloud(cloud_device,rot,trans,err_ps,trans_uniq,radius,err_p_size);

                // download new points to cloud
                cloud_device.download(cloud_RGB_T->points);

                double end = std::clock();
		        double time_taken = double(end-start) / double(CLOCKS_PER_SEC);
		        std::cout << "Time taken for the task :" << time_taken <<std::endl;
                std::cout<<"-------------------------------------------Deformation successful !!! --------------------------------------- :"<<std::endl;
                std::cout<<"======================="<<std::endl;

                // checking key points points after deformation
                // for(int i =0; i < keypoint_places.size(); i++){
                //     std::pair<double,double> poi = keypoint_places[i];
                //     std::cout<<cloud_RGB_T->at(poi.first,poi.second).x<<","<<cloud_RGB_T->at(poi.first,poi.second).y<<","<<cloud_RGB_T->at(poi.first,poi.second).z<<std::endl;
                // }

                // free memory
                free(rot);
                free(trans);
                free(err_ps);
                free(trans_uniq);

                // finally brodcast the error as map to odom transform - all matching key point must have an error (not point in exact location)
                if(New_frame_matches.size() == erronous_cod.size()){

                    double xx = R.at<double>(0,0);
                    double xy = R.at<double>(0,1);
                    double xz = R.at<double>(0,2);

                    double yx = R.at<double>(1,0);
                    double yy = R.at<double>(1,1);
                    double yz = R.at<double>(1,2);

                    double zx = R.at<double>(2,0);
                    double zy = R.at<double>(2,1);
                    double zz = R.at<double>(2,2);

                    tf2::Matrix3x3 tm(xx, xy, xz, yx, yy, yz, zx, zy, zz);

                    double x = t.at<double>(0,0);
                    double y = t.at<double>(1,0);
                    double z = t.at<double>(2,0);

                    tf2::Vector3 translation(x, y, z);

                    tf2::Transform trans(tm, translation);
                    geometry_msgs::TransformStamped error;
                    error.header.stamp = time;
                    error.header.frame_id = "map";
                    error.child_frame_id = "odom";
                    error.transform = tf2::toMsg(trans);

                    br.sendTransform(error);

                }



                not_the_same = 0;
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
            ros::Duration(1.0).sleep();
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
    ros::Rate r(20);
    while (ros::ok())
    {
    	ros::spinOnce();
    	r.sleep();     
     }

    return 0;

}
