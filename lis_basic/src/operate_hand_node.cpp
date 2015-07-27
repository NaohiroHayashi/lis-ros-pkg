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
    ros::NodeHandle n;
    ros::init(argc, argv, "operate_hand_node");
    std::cout << "Initializing node... " << std::endl;
    ros::Publisher pub_hook = n.advertise<std_msgs::String>("hook", 10);
    std_msgs::String pub_msg;
    
    pub_msg.data = "a";//right_hook open
    pub_hook.publish(pub_msg);
    ros::Duration(5.0).sleep();//wait time

    return 0;
}
