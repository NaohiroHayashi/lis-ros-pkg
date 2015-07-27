// ik_node.cpp
// Copyright 2014 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <iostream>
#include <lis_msgs/Joint_angles.h>
#include <lis_msgs/End_poses.h>
using namespace std;
//publisher
ros::Publisher pub_ik_;
//service
ros::ServiceClient left_client_, right_client_;

void calIK(const lis_msgs::End_poses& msg){
  //srv
  baxter_core_msgs::SolvePositionIK iksvc_l, iksvc_r;

  iksvc_l.request.pose_stamp.resize(1);
  iksvc_l.request.pose_stamp[0].header.stamp = ros::Time::now();
  iksvc_l.request.pose_stamp[0].header.frame_id = "base";
  iksvc_l.request.pose_stamp[0].pose.position = msg.left.position;
  iksvc_l.request.pose_stamp[0].pose.orientation = msg.left.orientation;
  iksvc_l.request.seed_mode = 2;

  iksvc_r.request.pose_stamp.resize(1);
  iksvc_r.request.pose_stamp[0].header.stamp = ros::Time::now();
  iksvc_r.request.pose_stamp[0].header.frame_id = "base";
  iksvc_r.request.pose_stamp[0].pose.position = msg.right.position;
  iksvc_r.request.pose_stamp[0].pose.orientation = msg.right.orientation;
  iksvc_r.request.seed_mode = 2;

  if (left_client_.call(iksvc_l)) {
      ROS_INFO("SUCCESS to left call service");
  }
  else {
      ROS_ERROR("FAILED to left call service");
  }

  if (iksvc_l.response.isValid[0]) {
      ROS_INFO("SUCCESS - Valide left Joint Sloution Found:");
  }
  else {
      ROS_ERROR("INVALID LEFT POSE");
  }

  if (right_client_.call(iksvc_r)) {
      ROS_INFO("SUCCESS to right call service");
  }
  else {
      ROS_ERROR("FAILED to right call service");
  }

  if (iksvc_r.response.isValid[0]) {
      ROS_INFO("SUCCESS - Valide right Joint Sloution Found:");
  }
  else {
      ROS_ERROR("INVALID RIGHT POSE");
  }

  lis_msgs::Joint_angles pub_joint_angles;

  if (iksvc_l.response.isValid[0] == 1 && iksvc_r.response.isValid[0] == 1) {
    for (int i = 0; i < 7; i++) {
      pub_joint_angles.left[i] = iksvc_l.response.joints[0].position[i];
      pub_joint_angles.right[i] = iksvc_r.response.joints[0].position[i];
    }
    pub_ik_.publish(pub_joint_angles);
    ROS_INFO("SUCCESS publish:");
  }
  else {
    ROS_ERROR("FAILED publish");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_node");
  ros::NodeHandle n;
  cout << "Initializing node... " << endl;

  //publiher
  pub_ik_ = n.advertise<lis_msgs::Joint_angles>("ik_topic", 10);
  //subscriber
  ros::Subscriber sub_endpos_topic = n.subscribe("end_pose_topic", 10, calIK);
  //service
  left_client_ = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/left/PositionKinematicsNode/IKService");
  right_client_ = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
  
  ros::spin(); //callback
  return 0;
}
