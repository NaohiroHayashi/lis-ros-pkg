// Copyright 2014 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
//ベース座標系殻見たdesk座標系をTFに送る
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
using namespace std;


int main(int argc, char** argv){
    ros::init(argc, argv, "tf_base2desk_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    
    ros::Rate rate(10.0);
    while (node.ok()){
    br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(180.0*M_PI/180.0, 0.0, 0.0*M_PI/180.0), tf::Vector3(0.5, 0.2, 0.035)), ros::Time::now(), "/base", "/desk"));
        rate.sleep();
    }
    return 0;
};

