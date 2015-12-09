//end_poseArray_nodeの処理を右左、分割して行う.
//サブスクライブするlis_msgs::End_PosesArrayのデータ数が左右で違っても大丈夫になっている.
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

class HandTransform{
 private:
  ros::NodeHandle n_;
  ros::Publisher pub_end_pose_;
  ros::Subscriber sub_target_pose_;
  tf::TransformListener listener_;

 public:
  HandTransform(){
    pub_end_pose_ = n_.advertise<lis_msgs::End_PosesArray>("end_poseArray_topic", 10);
    sub_target_pose_ = n_.subscribe("hand_pathArray_topic", 10, &HandTransform::target_pose2end_pose, this);
  }

  void target_pose2end_pose(const lis_msgs::End_PosesArray& msg){
    lis_msgs::End_PosesArray pub_msg;
    geometry_msgs::PoseStamped stamped_in, stamped_out;
    tf::TransformBroadcaster br;
    tf::StampedTransform rep,lep;
    ros::Time now;

    /*-------------------------------------------------------------------------*/
    /*--------------------------------GETDATA----------------------------------*/
    /*-------------------------------------------------------------------------*/
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

    /*-------------------------------------------------------------------------*/
    /*---------------------------------LEFT------------------------------------*/
    /*-------------------------------------------------------------------------*/
    for (int i = 0; i < msg.left.size(); i++) {
      now = ros::Time::now();

      //TFにbaseから見たtarget_pose（目標グリッパ位置）の座標系を登録
      br.sendTransform(tf::StampedTransform(tf::Transform(
                       tf::Quaternion(msg.left[i].orientation.x, msg.left[i].orientation.y,
                                      msg.left[i].orientation.z, msg.left[i].orientation.w),
                       tf::Vector3(msg.left[i].position.x,
                                   msg.left[i].position.y,
                                   msg.left[i].position.z)),
                       now , msg.header.frame_id, "/left_target_pose"));

      stamped_in = createPoseStampedMsg (pose_tf2gm(&lep), "/left_target_pose", now);

      try {
        listener_.waitForTransform("/base", "/left_target_pose", now, ros::Duration(1.0));
        listener_.transformPose ("/base", now, stamped_in, "left_target_pose", stamped_out);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s %d",ex.what(), i);
      }

     cout << "out"<< i << " " << stamped_out.header.frame_id << " " << stamped_out.pose << endl;
      pub_msg.left.push_back(stamped_out.pose);
    }

    /*-------------------------------------------------------------------------*/
    /*---------------------------------RIGHT-----------------------------------*/
    /*-------------------------------------------------------------------------*/
    for (int i = 0; i < msg.right.size(); i++) {
      now = ros::Time::now();

      br.sendTransform(tf::StampedTransform(tf::Transform(
                       tf::Quaternion(msg.right[i].orientation.x, msg.right[i].orientation.y,
                                      msg.right[i].orientation.z, msg.right[i].orientation.w),
                       tf::Vector3(msg.right[i].position.x,
                                   msg.right[i].position.y,
                                   msg.right[i].position.z)),
                       now , msg.header.frame_id, "/right_target_pose"));

      stamped_in = createPoseStampedMsg (pose_tf2gm(&rep), "/right_target_pose", now);

      try {
        listener_.waitForTransform("/base", "/right_target_pose", now, ros::Duration(1.0));
        listener_.transformPose ("/base", now, stamped_in, "/right_target_pose", stamped_out);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
      }

      pub_msg.right.push_back(stamped_out.pose);
    }

    /*-------------------------------------------------------------------------*/
    /*--------------------------------PUBLISH----------------------------------*/
    /*-------------------------------------------------------------------------*/
    ROS_INFO("publish data");
    pub_msg.header.frame_id = stamped_out.header.frame_id;
    pub_end_pose_.publish(pub_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_transform_node");

  cout << "Initializing node... " << endl;
  HandTransform handtransform;
  ros::spin(); //callback
  return 0;
}
