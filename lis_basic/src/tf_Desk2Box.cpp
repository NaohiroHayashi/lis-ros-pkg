// Copyright 2014 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
//ベース座標系殻見たdesk座標系をTFに送る
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
using namespace std;


int main(int argc, char** argv){
    ros::init(argc, argv, "tf_desk2box_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    
    ros::Rate rate(10.0);
    while (node.ok()){
    br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0), tf::Vector3(0.0, 0.16, 0)), ros::Time::now(), "/desk", "/box"));
        rate.sleep();
    }
    return 0;
};

