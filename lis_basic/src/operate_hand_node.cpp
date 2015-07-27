// Copyright 2015 Naohiro Hayashi
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
//-------------------------------------------------------//
//~ rostopic pub /hook std_msgs/String "a" 左手オープン10deg
//~ rostopic pub /hook std_msgs/String "b" 左手クローズ
//~ rostopic pub /hook std_msgs/String "c" 右手オープン10deg
//~ rostopic pub /hook std_msgs/String "d" 右手クローズ
//~ rostopic pub /hook std_msgs/String "e" 両手オープン10deg
//~ rostopic pub /hook std_msgs/String "f" 両手クローズ
//--------------------------------------------------------//

int main(int argc, char **argv)
{
    ros::init(argc, argv, "operate_hand_node");
    ros::NodeHandle n;

    std::cout << "Initializing node... " << std::endl;
    ros::Publisher pub_hook = n.advertise<std_msgs::String>("hook", 10);
    std_msgs::String pub_msg;
    
    //少し待たないと最初のコマンドが届かない
    ros::Duration(1.0).sleep();//wait time
    
    ROS_INFO("Left hand open");
    pub_msg.data = "a";//左手オープン10deg
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    ROS_INFO("Left hand close");
    pub_msg.data = "b";//左手クローズ
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    ROS_INFO("Right hand open");
    pub_msg.data = "c";//右手オープン10deg
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    ROS_INFO("Right hand close");
    pub_msg.data = "d";//右手クローズ
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    ROS_INFO("Both hand open");
    pub_msg.data = "e";//両手オープン10deg
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    ROS_INFO("Both hand close");
    pub_msg.data = "f";//右手クローズ
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    return 0;
}
