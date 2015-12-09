// trajectory_node.cpp
// Copyright 2015 Naohiro Hayashi <hayashi@taka.is.uec.ac.jp>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <lis_msgs/JointAnglesArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#define MAX_JOINT_VEL 0.05 //in radians/sec
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
using namespace std;

class TrajectoryClient{
 private:
  ros::NodeHandle n_;
  ros::Subscriber joint_sub_;
  Client *left_client_, *right_client_;

 public:
  TrajectoryClient(){
    joint_sub_ = n_.subscribe("ik_array_topic", 1, &TrajectoryClient::CallServer, this);

    //tell the joint trajectory action client that we want
    //to spin a thread by default
    left_client_ = new Client("/robot/limb/left/follow_joint_trajectory", true);
    right_client_= new Client("/robot/limb/right/follow_joint_trajectory", true);

    //wait for the action server to come up
    ROS_INFO("Waiting fot connecting with the server...");
    while(ros::ok() && !left_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the left joint_trajectory_action action server to come up");
    }
    while(ros::ok() && !right_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the right joint_trajectory_action action server to come up");
    }
    ROS_INFO("CONNECTED");

  }

  ~TrajectoryClient(){
     delete left_client_, right_client_;
  }

  //figure out where the arm is now
  void get_current_joint_angles(double current_angles[17]){
    sensor_msgs::JointStateConstPtr state_msg =
    ros::topic::waitForMessage<sensor_msgs::JointState>
    ("/robot/joint_states");

    //extract the joint angles from it
    for (int i = 0; i < 17; i++) {
      current_angles[i] = state_msg->position[i];
    }
  }

  void SetClient(const std::vector<lis_msgs::JointAngles>& joint_angles, const string limb, trajectory_msgs::JointTrajectory& traj){
    unsigned int n_joints = 7;
    std::vector<std::string> joint_names(n_joints);

    // One point trajectory
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n_joints,0.0);
    point.velocities.resize(n_joints,0.0);
    point.accelerations.resize(n_joints,0.0);

    std::vector<trajectory_msgs::JointTrajectoryPoint> points(joint_angles.size()+1, point);

    //get the current joint angles
    double current_angles[17];
    get_current_joint_angles(current_angles);

    //current_angles = 'head_nod', 'head_pan',
    //                 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2',
    //                 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2',
    //                 'torso_t0'
    if (limb == "left") {
      points[0].positions[0] = current_angles[4];
      points[0].positions[1] = current_angles[5];
      points[0].positions[2] = current_angles[2];
      points[0].positions[3] = current_angles[3];
      points[0].positions[4] = current_angles[6];
      points[0].positions[5] = current_angles[7];
      points[0].positions[6] = current_angles[8];

      // First, the joint names, which apply to all waypoints
      string left_joint_names[7] = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
      for (int i = 0; i < n_joints; i++) joint_names[i] = left_joint_names[i];
    }
    else if (limb == "right") {
      points[0].positions[0] = current_angles[11];
      points[0].positions[1] = current_angles[12];
      points[0].positions[2] = current_angles[9];
      points[0].positions[3] = current_angles[10];
      points[0].positions[4] = current_angles[13];
      points[0].positions[5] = current_angles[14];
      points[0].positions[6] = current_angles[15];

      // First, the joint names, which apply to all waypoints
      string right_joint_names[7] = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
      for (int i = 0; i < n_joints; i++) joint_names[i] = right_joint_names[i];
    }
    else {
      ROS_ERROR("In function, input left or right");
      return;
    }

    points[0].time_from_start = ros::Duration(0.0);
    double time_from_start = 0.0;
    //fill in the joint positions
    for (int i = 0; i < joint_angles.size(); i++) {
      for (int j = 0; j < n_joints; j++) {
        points[i+1].positions[j] = joint_angles[i].joints[j];
      }

      //compute a desired time for this trajectory point using a max
      double max_joint_move = 0;
      for (int j = 0; j < n_joints; j++) {
        double joint_move = fabs(points[i+1].positions[j] - points[i].positions[j]);
        if (joint_move > max_joint_move) max_joint_move = joint_move;
      }
      double seconds = max_joint_move/MAX_JOINT_VEL;
      ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
      time_from_start += seconds;
      points[i+1].time_from_start = ros::Duration(time_from_start);
    }

    traj.joint_names=joint_names;
    traj.header.stamp = ros::Time::now() + ros::Duration(0.0);  //when to start the trajectory
    traj.points=points;
  }

  void CallServer(const lis_msgs::JointAnglesArray& msg){
    control_msgs::FollowJointTrajectoryGoal left_goal, right_goal; // Action goals

    if (msg.left.size() != 0 && msg.right.size() != 0) {
      SetClient(msg.left, "left", left_goal.trajectory);
      SetClient(msg.right, "right", right_goal.trajectory);

      ROS_INFO("Sending goal to both joint_trajectory_action");
      left_client_->sendGoal(left_goal);
      right_client_->sendGoal(right_goal);
      ROS_ASSERT_MSG(left_client_->waitForResult(),"Left goal not reached.");
      ROS_ASSERT_MSG(right_client_->waitForResult(),"Right goal not reached.");
      ROS_INFO("Goal reached!");
      ROS_INFO_STREAM("action client state is " << left_client_->getState().toString());
      ROS_INFO_STREAM("action client state is " << right_client_->getState().toString());
    }
    else if (msg.left.size() != 0 && msg.right.size() == 0) {
      SetClient(msg.left, "left", left_goal.trajectory);
      ROS_INFO("Sending goal to left joint_trajectory_action");
      left_client_->sendGoal(left_goal);
      ROS_ASSERT_MSG(left_client_->waitForResult(),"Left goal not reached.");
      ROS_INFO("Goal reached!");
      ROS_INFO_STREAM("action client state is " << left_client_->getState().toString());
    }
    else if (msg.left.size() == 0 && msg.right.size() != 0) {
      SetClient(msg.right, "right", right_goal.trajectory);
      ROS_INFO("Sending goal to right joint_trajectory_action");
      right_client_->sendGoal(right_goal);
      ROS_ASSERT_MSG(right_client_->waitForResult(),"Right goal not reached.");
      ROS_INFO_STREAM("action client state is " << right_client_->getState().toString());
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
