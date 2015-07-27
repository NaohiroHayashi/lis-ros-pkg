#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#include <lis_msgs/Joint_angles.h>
#define SPEED_RATIO 0.1
#define CMD_TIMEOUT 1
using namespace std;

class ExeNode{
 private:
  ros::NodeHandle n;
  ros::Publisher pub_rate, pub_speed_ratio_l,pub_joint_cmd_l, pub_joint_cmd_timeout_l, pub_speed_ratio_r, pub_joint_cmd_r, pub_joint_cmd_timeout_r;
  ros::Subscriber sub_joint;
  baxter_core_msgs::JointCommand joint_cmd_l,joint_cmd_r;
  std_msgs::Float64 speed_ratio, cmd_timeout;
  std_msgs::UInt16 rate;

 public:
  ExeNode(){
    //publisher
    pub_rate = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 10);

    pub_speed_ratio_l = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 10);
    pub_joint_cmd_l = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
    pub_joint_cmd_timeout_l = n.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 10);

    pub_speed_ratio_r = n.advertise<std_msgs::Float64>("robot/limb/right/set_speed_ratio", 10);
    pub_joint_cmd_r = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 10);
    pub_joint_cmd_timeout_r = n.advertise<std_msgs::Float64>("robot/limb/right/joint_command_timeout", 10);

    //subscriber
    sub_joint = n.subscribe("ik_topic", 10, &ExeNode::moveArm, this);

    joint_cmd_l.names.resize(7);
    joint_cmd_l.command.resize(7);
    joint_cmd_r.names.resize(7);
    joint_cmd_r.command.resize(7);
    speed_ratio.data = SPEED_RATIO;//joint speed
    cmd_timeout.data = CMD_TIMEOUT;
    joint_cmd_l.mode = 1;//position-mode
    joint_cmd_r.mode = 1;//position-mode
    rate.data = 100;

    string joint_names_l[7] = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    string joint_names_r[7] = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};

    for (int i = 0; i < 7; i++) {
      joint_cmd_l.names[i] = joint_names_l[i];
      joint_cmd_r.names[i] = joint_names_r[i];
    }
  }

  void moveArm(const lis_msgs::Joint_angles& joint_angles){
    for (int i = 0; i < 7; i++) {
      joint_cmd_l.command[i] = joint_angles.left[i];
      joint_cmd_r.command[i] = joint_angles.right[i];
    }
    for (int i = 0; i < 10 ;i++) {
      pub_rate.publish(rate); //The rate at which the joints are published can be controlled by publishing a frequency on this topic. Default rate is 100Hz; Maximum is 1000Hz
      pub_joint_cmd_timeout_l.publish(cmd_timeout);
      pub_joint_cmd_timeout_r.publish(cmd_timeout);
      pub_speed_ratio_l.publish(speed_ratio); //set joint speed default =0.3 range= 0.0-1.0
      pub_speed_ratio_r.publish(speed_ratio); //set joint speed default =0.3 range= 0.0-1.0
      pub_joint_cmd_l.publish(joint_cmd_l);
      pub_joint_cmd_r.publish(joint_cmd_r);
      ros::Rate loop_rate(5);
      loop_rate.sleep(); //sleep
    }
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exe_node");
    cout << "Initializing node... " << endl;
    ExeNode exe_node;
    ros::spin();//callback
    return 0;
}
