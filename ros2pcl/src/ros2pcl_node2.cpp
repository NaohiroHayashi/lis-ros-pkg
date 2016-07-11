// ros2pcl_node2.cpp
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>//for box filter
#include <pcl/filters/passthrough.h>// for XYZ-range filter
#include <iostream>
using namespace std;
typedef sensor_msgs::PointCloud2 SMPC2;

class ROS2PCL{
  private:
	ros::NodeHandle n_;
	ros::Publisher pub_topic_;
	ros::Subscriber sub_;
	sensor_msgs::PointCloud2Ptr pub_filtered_;
	tf::TransformListener tflistener_;
    
  public:
    ROS2PCL(){
		pub_topic_ = n_.advertise<sensor_msgs::PointCloud2> ("downsampling_topic", 1);
		sub_ = n_.subscribe("/camera/depth/points", 5, &ROS2PCL::callBack, this);
		pub_filtered_.reset (new SMPC2);
    }

    //http://wiki.ros.org/ja/pcl/Tutorials
    void callBack(const sensor_msgs::PointCloud2ConstPtr& msg){
    	//pcl::VoxelGrid<type> can use type of sensor_msgs::PointCloud2 and pcl::PointCloud.
    	//pcl::ApproximateVoxelGrid<type> can use only type of pcl::PointCloud.
    	//http://docs.pointclouds.org/1.6.0/functions_func.html
		pcl::VoxelGrid<SMPC2> sor;
		sor.setInputCloud (msg);
		sor.setLeafSize (0.01, 0.01, 0.01);
		sor.filter (*pub_filtered_);

		transformCloud ();

		pub_topic_.publish (pub_filtered_);
    }

	//Transform coordine of pub_filtered_ points
    void transformCloud(void){
        //Transform coordinates of cloud
        tflistener_.waitForTransform("/base", pub_filtered_->header.frame_id, pub_filtered_->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud("/base", *pub_filtered_, *pub_filtered_, tflistener_);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "ros2pcl_node");
    cout << "Initializing node... " << endl;
    ROS2PCL ros2pcl;
    ros::spin();
    
    return 0;
}
