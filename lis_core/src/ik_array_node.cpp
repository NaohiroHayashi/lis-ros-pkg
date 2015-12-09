//(C)2014 Naohiro Hayshi 2015/06/17
//左右のデータ配列数が違っていても解けるように改造
#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <lis_msgs/JointAnglesArray.h>
#include <lis_msgs/End_PosesArray.h>
using namespace std;

//publisher
ros::Publisher pub_rate_array_, pub_false_, pub_success_;
//serviceclient
ros::ServiceClient client_l_;
ros::ServiceClient client_r_;

void calcurate_IK(const lis_msgs::End_PosesArray &msg){
  std_msgs::Int32MultiArray res_num_l, res_num_r; //Record operation numbers which cant calcurate IK
  lis_msgs::End_PosesArray  pub_false_msg, pub_success_msg;
  lis_msgs::JointAnglesArray joint_angles_array;

  //srvmsg
  baxter_core_msgs::SolvePositionIK iksvc_l, iksvc_r;
  pub_false_msg.header =  pub_success_msg.header = msg.header;

  //サービスが成功したかをチェックするためのフラグ
  int flag_l = 0;
  int flag_r = 0;

  //--------------------------------------------------------------------//
  //--------------------------------LEFT--------------------------------//
  //--------------------------------------------------------------------//
  if (msg.left.size() != 0) {
    //配列サイズをメッセージと同じサイズに変更
    joint_angles_array.left.resize(msg.left.size());
    iksvc_l.request.pose_stamp.resize(msg.left.size());
    iksvc_l.response.joints.resize(msg.left.size());

    //目標手先位置姿勢をクライアントメッセージに格納
    for (int i = 0; i < msg.left.size(); i++) {
      iksvc_l.request.pose_stamp[i].header = msg.header;
      iksvc_l.request.pose_stamp[i].pose = msg.left[i];
    }

    //サービスを利用できたかチェック
    if (client_l_.call(iksvc_l)) {
      ROS_INFO("SUCCESS to left call service");
    }
    else {
      ROS_ERROR("FAILED to left call service");
    }

    //IKが解けたかチェック
    for (int i = 0; i < msg.left.size(); i++) {
      if(iksvc_l.response.isValid[i]){
        ROS_INFO("SUCCESS left- Valide Joint Sloution Found %d", i);
        pub_success_msg.left.push_back(msg.left[i]);
      }
      else {
        ROS_ERROR("INVALID POSE LEFT %d", i);
        flag_l = 1;//1回でもエラーが出たら、フラグを１にする
        res_num_l.data.push_back(i);
        pub_false_msg.left.push_back(msg.left[i]);
      }
    }

    //Ik
    if (flag_l == 0) {
      for (int i = 0; i < msg.left.size(); i++) {
        for (int j = 0; j < 7; j++) {
          joint_angles_array.left[i].joints[j] = iksvc_l.response.joints[i].position[j];
        }
      }
    }
  }

  //--------------------------------------------------------------------//
  //--------------------------------RIGHT-------------------------------//
  //--------------------------------------------------------------------//
  if (msg.right.size() != 0) {
    //配列サイズをメッセージと同じサイズに変更
    joint_angles_array.right.resize(msg.right.size());
    iksvc_r.request.pose_stamp.resize(msg.right.size());
    iksvc_r.response.joints.resize(msg.right.size());

    for(int i = 0; i < msg.right.size(); i++) {
      iksvc_r.request.pose_stamp[i].header = msg.header;
      iksvc_r.request.pose_stamp[i].pose = msg.right[i];
    }

    if (client_r_.call(iksvc_r)) {
      ROS_INFO("SUCCESS to right call service");
    }
    else {
      ROS_ERROR("FAILED to right call service");
    }

    //IKが解けたかチェック
    for (int i = 0; i < msg.right.size(); i++) {
      if (iksvc_r.response.isValid[i]) {
        ROS_INFO("SUCCESS right- Valide Joint Sloution Found %d", i);
        pub_success_msg.right.push_back(msg.right[i]);
      }
      else {
        ROS_ERROR("INVALID POSE RIGHT %d", i);
        flag_r = 1;//1回でもエラーが出たら、フラグを１にする
        res_num_r.data.push_back(i);
        pub_false_msg.right.push_back(msg.right[i]);
      }
    }

    //IK
    if (flag_r == 0) {
      for (int i = 0; i < msg.right.size(); i++) {
        for (int j = 0; j < 7; j++) {
          joint_angles_array.right[i].joints[j] = iksvc_r.response.joints[i].position[j];
        }
      }
    }
  }

  //--------------------------------------------------------------------//
  //--------------------------PUBLISH RESULT----------------------------//
  //--------------------------------------------------------------------//
  //Publish True or False data
  pub_false_.publish(pub_false_msg);
  pub_success_.publish(pub_success_msg);

  //IKが全部解けたか、フラグでチェック　大丈夫ならパブリッシュmsgに格納してパブリッシュ
  if (flag_l == 0 && flag_r == 0) {
    ROS_INFO("SUCCESS TO IK CALCURATION");
    pub_rate_array_.publish(joint_angles_array);
  }
  else {
    ROS_ERROR("FAILED TO IK CALCULATION");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_array_node");
  ros::NodeHandle n;
  cout << "Initializing node... " << endl;

  //publiher
  pub_rate_array_ = n.advertise<lis_msgs::JointAnglesArray>("ik_array_topic", 10);
  pub_false_ = n.advertise<lis_msgs::End_PosesArray>("plan_result_topic", 10);
  pub_success_ = n.advertise<lis_msgs::End_PosesArray>("plan_result2_topic", 10);
  //subscriber
  ros::Subscriber sub_endpos_topic = n.subscribe("end_poseArray_topic", 10, calcurate_IK);
  //service
  client_l_ = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/left/PositionKinematicsNode/IKService");
  client_r_ = n.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");

  ros::spin(); //callback
  return 0;
}
