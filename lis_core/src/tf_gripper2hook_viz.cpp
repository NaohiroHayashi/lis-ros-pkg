// Copyright 2015 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
using namespace std;

ros::Subscriber sub_hook_;
visualization_msgs::Marker l_hook_, r_hook_;
string path_lclose_, path_lopen_;
string path_rclose_, path_ropen_;

void hookCb(const std_msgs::String msg){
  if ("a" == msg.data) l_hook_.mesh_resource = path_lopen_;
  else if ("b" == msg.data) l_hook_.mesh_resource = path_lclose_;
  else if ("c" == msg.data) r_hook_.mesh_resource = path_ropen_;
  else if ("d" == msg.data) r_hook_.mesh_resource = path_rclose_;
  else if ("e" == msg.data) {
    l_hook_.mesh_resource = path_lopen_;
    r_hook_.mesh_resource = path_ropen_;
  }
  else if ("f" == msg.data) {
    l_hook_.mesh_resource = path_lclose_;
    r_hook_.mesh_resource = path_rclose_;
  }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_gripper2hook");
    ros::NodeHandle n;

    ros::Publisher pub_l_hook, pub_r_hook;
    tf::TransformBroadcaster br;
    tf::Transform rtransform,ltransform;
    tf::Quaternion q;

    pub_l_hook = n.advertise<visualization_msgs::Marker>("display_l_hook", 1);
    pub_r_hook = n.advertise<visualization_msgs::Marker>("display_r_hook", 1);
    sub_hook_ = n.subscribe("/hook", 10, &hookCb);

    //stl path
    path_lclose_ += "file:///";
    path_lclose_ += getenv("HOME");
    path_lclose_ +="/catkin_ws/src/lis_core/src/stl/LIS_lclose_hook.stl";
    path_lopen_ += "file:///";
    path_lopen_ += getenv("HOME");
    path_lopen_ +="/catkin_ws/src/lis_core/src/stl/LIS_lopen_hook.stl";
    path_rclose_ += "file:///";
    path_rclose_ += getenv("HOME");
    path_rclose_ +="/catkin_ws/src/lis_core/src/stl/LIS_rclose_hook.stl";
    path_ropen_ += "file:///";
    path_ropen_ += getenv("HOME");
    path_ropen_ +="/catkin_ws/src/lis_core/src/stl/LIS_ropen_hook.stl";
    
    //left
    l_hook_.header.frame_id = "left_gripper";
    l_hook_.header.stamp = ros::Time::now();
    l_hook_.type = visualization_msgs::Marker::MESH_RESOURCE;
    l_hook_.action = visualization_msgs::Marker::ADD;
    l_hook_.pose.position.x = 0.0;
    l_hook_.pose.position.y = 0.0;
    l_hook_.pose.position.z = 0.0;
    l_hook_.pose.orientation.x = 0.0;
    l_hook_.pose.orientation.y = 0.0;
    l_hook_.pose.orientation.z = 0.0;
    l_hook_.pose.orientation.w = 1.0;
    l_hook_.scale.x = 1.0;
    l_hook_.scale.z = 1.0;
    l_hook_.scale.y = 1.0;
    l_hook_.color.r = 0.3f;
    l_hook_.color.g = 0.3f;
    l_hook_.color.b = 0.3f;
    l_hook_.color.a = 1.0;
    l_hook_.lifetime = ros::Duration();
    l_hook_.mesh_use_embedded_materials = true;
    l_hook_.mesh_resource = path_lclose_;
    
    //right
    r_hook_ = l_hook_;
    r_hook_.header.frame_id = "right_gripper";
    r_hook_.mesh_resource = path_rclose_;
    
    q.setRPY(0.0, 0.0, 0.0);
    rtransform.setRotation(q);
    rtransform.setOrigin( tf::Vector3(-0.005, 0.000, 0.12) );
    ltransform.setRotation(q);
    ltransform.setOrigin( tf::Vector3(-0.002, 0.008, 0.12) );
    
    ros::Rate rate(10.0);
    
    while (n.ok()){
      br.sendTransform(tf::StampedTransform(rtransform, ros::Time::now(), "right_gripper", "right_endgripper2"));
      br.sendTransform(tf::StampedTransform(ltransform, ros::Time::now(), "left_gripper", "left_endgripper2"));
      l_hook_.header.stamp = r_hook_.header.stamp = ros::Time::now();
      pub_l_hook.publish(l_hook_);
      pub_r_hook.publish(r_hook_);
      ros::spinOnce();
      rate.sleep();
    }
    
    return 0;
};
