// ros2pcl_node.cpp
// Copyright 2016 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/approximate_voxel_grid.h>//for box filter
#include <pcl/filters/passthrough.h>// for XYZ-range filter
#include <iostream>
using namespace std;
typedef pcl::PointXYZ PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLPointCloud;
#define BOX_SIZE 0.05f //[m]

class ROS2PCL{
  private:
	ros::NodeHandle n_;
	ros::Publisher pub_topic_;
	ros::Subscriber sub_;
    
	sensor_msgs::PointCloud2Ptr pub_filtered_;
	pcl::PointCloud<PCLPointType>::Ptr cloud_filtered_;
	tf::TransformListener tflistener_;
    
  public:
    ROS2PCL(){
		pub_topic_ = n_.advertise<sensor_msgs::PointCloud2> ("filtered_topic", 1);
		sub_ = n_.subscribe("/camera/depth/points", 1, &ROS2PCL::callBack, this);
		pub_filtered_.reset (new sensor_msgs::PointCloud2);
		cloud_filtered_.reset (new PCLPointCloud);
        ros::Duration(0.5).sleep(); // sleep for registration of tf
    }

    //http://wiki.ros.org/ja/pcl/Tutorials
    void callBack(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg (*msg, *cloud_filtered_);// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud

        filterBOX ();
        filterXYZ ();
        transformCloud ();

        pcl::toROSMsg (*cloud_filtered_, *pub_filtered_);//Convert pcl/PointCloud to sensor_msgs/PointCloud2
        pub_topic_.publish (pub_filtered_);
    }

    void filterBOX(void){
        pcl::ApproximateVoxelGrid<PCLPointType> sor;

        sor.setInputCloud (cloud_filtered_);
        sor.setLeafSize (BOX_SIZE, BOX_SIZE, BOX_SIZE);//Down sampling the Pointclouds as 0.01ｍ box
        sor.filter (*cloud_filtered_);
    }

	void filterXYZ(void){
		pcl::PassThrough<PCLPointType> pass;

		pass.setInputCloud (cloud_filtered_);
		pass.setFilterFieldName ("z");// filtering by z-range
		pass.setFilterLimits (0.05, 2.0);//[m]
		pass.filter (*cloud_filtered_);
	}

	//Transform coordine of cloud_filtered_ points
    void transformCloud(void){
    	sensor_msgs::PointCloud2Ptr smpc2_native_cloud (new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Ptr smpc2_transformed_cloud (new sensor_msgs::PointCloud2);

        pcl::toROSMsg (*cloud_filtered_, *smpc2_native_cloud);

        tflistener_.waitForTransform("/base", cloud_filtered_->header.frame_id, cloud_filtered_->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud("/base", *smpc2_native_cloud, *smpc2_transformed_cloud, tflistener_);

        pcl::fromROSMsg (* smpc2_transformed_cloud, *cloud_filtered_);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "ros2pcl_node");
    cout << "Initializing node... " << endl;

    ROS2PCL ros2pcl;
    ros::spin();
    
    return 0;
}
