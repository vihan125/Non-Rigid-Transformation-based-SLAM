#include "odom.hpp"

RegisterOdom::RegisterOdom()
{
	//Assign subscriber
	this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&RegisterOdom::processPointCloud,this);
	this->publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/map", 10);
	this->client = this->nh.serviceClient<registration::getTransform>("get_Transform");

	// global clouds
	result_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    result_rgb->is_dense = false;
	globaltransform = Eigen::Matrix4f::Identity();

}

void RegisterOdom::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    int time = cloud_msg->header.stamp.sec;

    pcl::PCLPointCloud2 pcl_pc2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, *cloud_RGB);

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

            pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud (transformed_source_RGB);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*temp);

            result_rgb->operator+=(*temp);
            
            std::cout<<"len :"<<temp->size()<<std::endl;
            publisher.publish(result_rgb);

        }else{

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            // sor.setInputCloud (transformed_source_RGB);
            sor.setInputCloud (cloud_RGB);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*temp);
            std::cout<<"unable to find a transformation"<<std::endl;
            result_rgb->operator+=(*temp);
            std::cout<<"len :"<<temp->size()<<std::endl;
            publisher.publish(result_rgb);	
        }
    }
    else{
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

            pcl::transformPointCloud(*cloud_RGB,*transformed_source_RGB,globaltransform);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud (transformed_source_RGB);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*temp);

            pcl::copyPointCloud(*temp,*result_rgb);
            
            std::cout<<"len :"<<temp->size()<<std::endl;
            publisher.publish(result_rgb);
    }

    }
}