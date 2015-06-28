// trajectory_node.cpp
// Copyright 2015 Naohiro Hayashi <hayashi@taka.is.uec.ac.jp>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <lis_msgs/JointAnglesArray.h>
#include <string>
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
using namespace std;

class TrajectoryClient{
 private:
  ros::NodeHandle n_;
  ros::Subscriber joint_sub_;

 public:
  TrajectoryClient(){
    joint_sub_ = n_.subscribe("ik_array_node", 1, &TrajectoryClient::CallServer, this);
  }

  void SetClient(const std::vector<lis_msgs::JointAngles>& joint_angles, const string limb, trajectory_msgs::JointTrajectory& traj){
    unsigned int n_joints = 7;
    std::vector<std::string> joint_names(n_joints);

    // One point trajectory
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);

    if (limb == "left") {
      string left_joint_names[7] = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
      for (int i = 0; i < n_joints; i++) joint_names[i] = left_joint_names[i];
    }
    else if (limb == "right") {
      string right_joint_names[7] = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
      for (int i = 0; i < n_joints; i++) joint_names[i] = right_joint_names[i];
    }
    else {
      ROS_ERROR("In function, input left or right");
      return;
    }

    std::vector<trajectory_msgs::JointTrajectoryPoint> points(joint_angles.size(), point);

    for (int i = 0; i < points.size(); i++){
      for (int j = 0; j < n_joints; j++){
        points[i].positions[j] = joint_angles[i].joints[j];
      }
      points[i].time_from_start = ros::Duration(5*i+5);
    }

    traj.joint_names=joint_names;
    traj.header.stamp = ros::Time(0); // start inmediately
    traj.points=points;
  }

  void CallServer(const lis_msgs::JointAnglesArray& msg){
    Client left_client("/robot/limb/left/follow_joint_trajectory", true);
    Client right_client("/robot/limb/right/follow_joint_trajectory", true);

    ROS_INFO("Waiting fot connecting with the server...");
    ROS_ASSERT_MSG(left_client.waitForServer(ros::Duration(10)),"Timeout. Left server not available.");
    ROS_ASSERT_MSG(right_client.waitForServer(ros::Duration(10)),"Timeout. Right server not available.");
    ROS_INFO("CONNECTED");

    // Action goals
    control_msgs::FollowJointTrajectoryGoal left_goal, right_goal;
    ros::Time now = ros::Time::now();

    if (msg.left.size() != 0 && msg.right.size() != 0) {
      SetClient(msg.left, "left", left_goal.trajectory);
      SetClient(msg.right, "right", right_goal.trajectory);
      left_goal.trajectory.header.stamp  = right_goal.trajectory.header.stamp = now + ros::Duration(5.0); // start 1s after now
      left_client.sendGoal(left_goal);
      right_client.sendGoal(right_goal);
      ROS_ASSERT_MSG(left_client.waitForResult(),"Left goal not reached.");
      ROS_ASSERT_MSG(right_client.waitForResult(),"Right goal not reached.");
      ROS_INFO("Goal reached!");
      ROS_INFO_STREAM("action client state is " << left_client.getState().toString());
      ROS_INFO_STREAM("action client state is " << right_client.getState().toString());
    }
    else if (msg.left.size() != 0 && msg.right.size() == 0) {
      SetClient(msg.left, "left", left_goal.trajectory);
      left_goal.trajectory.header.stamp  = now + ros::Duration(5.0); // start 1s after now
      left_client.sendGoal(left_goal);
      ROS_ASSERT_MSG(left_client.waitForResult(),"Left goal not reached.");
      ROS_INFO("Goal reached!");
      ROS_INFO_STREAM("action client state is " << left_client.getState().toString());
    }
    else if (msg.left.size() == 0 && msg.right.size() != 0) {
      SetClient(msg.right, "right", right_goal.trajectory);
      right_goal.trajectory.header.stamp = now + ros::Duration(5.0); // start 1s after now
      right_client.sendGoal(right_goal);
      ROS_ASSERT_MSG(right_client.waitForResult(),"Right goal not reached.");
      ROS_INFO_STREAM("action client state is " << right_client.getState().toString());
    }
    else {
      ROS_ERROR("No data in msg");
      return;
    }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "joint_trajectory_client");
  TrajectoryClient trajectoryclient;
  ros::spin();
}
