#include <iostream>
#include "ros/ros.h"


#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

class SubscribeProcessPublish
{
    public:
        SubscribeProcessPublish()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/camera/depth/points",5,&SubscribeProcessPublish::processPointCloudMeasurement,this);

            //Assign publisher
            this->publisher = this->nh.advertise<pcl::PCLPointCloud2>("output",1);
        }

        void processPointCloudMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg){
            //std::cout << "Received Point cloud measurement with seq ID" << cloud_msg->header.seq << std::endl;
            //define a new container for the data
            pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());

            //define a voxel grid
            pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;

            //set input to cloud
            voxelGrid.setInputCloud(cloud_msg);

            //set the leaf size (x,y,z)
            voxelGrid.setLeafSize(0.1,0.1,0.1);

            //apply the filter to dereferenced cloud voxel
            voxelGrid.filter(*cloudVoxel);
            // publish the data
            this->publisher.publish(*cloudVoxel);
        }

    private: 
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
};

class NoteDownClouds
{
    public:
        NoteDownClouds()
        {
            //Assign subscriber
            this->subscriber = this->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",5,&NoteDownClouds::processPointCloud,this);

        }

        void processPointCloud(const sensor_msgs::PointCloud2 cloud_msg){
            int width = cloud_msg.width;
            int height = cloud_msg.height;
	    
            std::cout << "width:" << width<< std::endl;
	        std::cout << "height" << height<< std::endl;

            //sensor_msgs::PointField [] fields = cloud_msg.fields;
            //int len = sizeof(cloud_msg.fields[0].name);
            //std::cout << "one point length" << cloud_msg.point_step<< std::endl;
            for (int i =0 ; i<5;i++){
		    std::basic_string<char> name = cloud_msg.fields[i].name;
		    int offset = cloud_msg.fields[i].offset;
		    int count = cloud_msg.fields[i].count;
		    int d_type = cloud_msg.fields[i].datatype;

		    //std::cout << "fields: " << fields[0]<< std::endl;
		    std::cout << "name: " << name << std::endl;

		    std::cout << "offset: " << offset<< std::endl;
		    std::cout << "count: " << count<< std::endl;
		    std::cout << "dataType:" << d_type<< std::endl;
            }

            std::cout << "*************************************************************"<< std::endl;
            //define a new container for the data
	   	
        }

    private: 
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
};

int main(int argc, char **argv)
{
    //initialize the node
    ros::init(argc,argv,"process_PointCloud");
    std::cout << "Process_PointCloud initialized" << std::endl;

    // handle ROS communication events
    SubscribeProcessPublish PC_obj1;
    //NoteDownClouds PC_obj2;
    ros::spin();

    return 0;
}
