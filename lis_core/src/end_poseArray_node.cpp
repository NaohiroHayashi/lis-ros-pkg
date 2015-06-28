#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <lis_msgs/End_PosesArray.h>
#include <geometry_msgs/PoseArray.h>
#include <lis_msgs/lis.h>
#include <msg_helpers.h>
using namespace std;
using namespace object_manipulator;
using namespace msg;

class EndPoseArray{
 private:
  ros::NodeHandle n_;
  ros::Publisher pub_end_pose_;
  ros::Subscriber sub_target_pose_;
  tf::TransformListener listener_;
  
 public:
  EndPoseArray(){
    pub_end_pose_ = n_.advertise<lis_msgs::End_PosesArray>("end_poseArray_topic", 10);
    sub_target_pose_ = n_.subscribe("hand_pathArray_topic", 10, &EndPoseArray::target_pose2end_pose, this);
  }

  void target_pose2end_pose(const lis_msgs::End_PosesArray msg){
    lis_msgs::End_PosesArray pubmsg;
    geometry_msgs::PoseStamped stamped_in, stamped_out;
    tf::TransformBroadcaster br;
    tf::StampedTransform rep,lep;

    try {
      listener_.lookupTransform("/left_endgripper2", "/left_gripper", ros::Time(0), lep);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    try {
      listener_.lookupTransform("/right_endgripper2", "/right_gripper", ros::Time(0), rep);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    for (int i=0; i<msg.left.size(); i++) {
      ros::Time now = ros::Time::now();
      
      //TFにbaseから見たtarget_pose（目標グリッパ位置）の座標系を登録
      br.sendTransform(tf::StampedTransform(tf::Transform(
                       tf::Quaternion(msg.left[i].orientation.x, msg.left[i].orientation.y,
                                      msg.left[i].orientation.z, msg.left[i].orientation.w),
                       tf::Vector3(msg.left[i].position.x, 
                                   msg.left[i].position.y, 
                                   msg.left[i].position.z)), 
                       now , "/world3", "/left_target_pose"));
      
      br.sendTransform(tf::StampedTransform(tf::Transform(
                       tf::Quaternion(msg.right[i].orientation.x, msg.right[i].orientation.y,
                                      msg.right[i].orientation.z, msg.right[i].orientation.w),
                       tf::Vector3(msg.right[i].position.x,
                                   msg.right[i].position.y,
                                   msg.right[i].position.z)),
                       now , "/world3", "/right_target_pose"));
      
      stamped_in = createPoseStampedMsg (pose_tf2gm(&lep), "/left_target_pose", now);
      
      try {
        listener_.waitForTransform("/base", "/left_target_pose", now, ros::Duration(1.0));
        listener_.transformPose ("/base", now, stamped_in, "left_target_pose", stamped_out);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s %d",ex.what(), i);
      }
      
      pubmsg.left.push_back(stamped_out.pose);
      
      stamped_in = createPoseStampedMsg (pose_tf2gm(&rep), "/right_target_pose", now);
      
      try {
        listener_.waitForTransform("/base", "/right_target_pose", now, ros::Duration(1.0));
        listener_.transformPose ("/base", now, stamped_in, "/right_target_pose", stamped_out);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
      }
      
      pubmsg.right.push_back(stamped_out.pose);
      
      br.sendTransform(tf::StampedTransform(tf::Transform(
        tf::Quaternion(pubmsg.left[i].orientation.x, pubmsg.left[i].orientation.y, pubmsg.left[i].orientation.z, pubmsg.left[i].orientation.w),
        tf::Vector3(pubmsg.left[i].position.x, pubmsg.left[i].position.y, pubmsg.left[i].position.z)
      ), ros::Time::now() , "/base", "/left_end_pose"));

      br.sendTransform(tf::StampedTransform(tf::Transform(
        tf::Quaternion(pubmsg.right[i].orientation.x, pubmsg.right[i].orientation.y, pubmsg.right[i].orientation.z, pubmsg.right[i].orientation.w),
        tf::Vector3(pubmsg.right[i].position.x, pubmsg.right[i].position.y, pubmsg.right[i].position.z)
      ), ros::Time::now() , "/base", "/right_end_pose"));
    }
    
    pubmsg.header.frame_id = stamped_out.header.frame_id ;
    pub_end_pose_.publish(pubmsg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "end_poseArray_node");

  cout << "Initializing node... " << endl;
  EndPoseArray endposearray;
  ros::spin(); //callback
  return 0;
}
