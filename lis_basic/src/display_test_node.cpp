// display_test_node.cpp
// Copyright 2015 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
#include <ros/ros.h>
#include <iostream>
#include <baxter_core_msgs/EndpointState.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
//
ros::Publisher pub_left_path, pub_right_path;//publish
ros::Subscriber sub_left_path, sub_right_path;//subscribe
nav_msgs::Path pub_lmsg,pub_rmsg;//path用変数

//左手の位置姿勢が更新されると呼ばれる関数
void left_callback(const baxter_core_msgs::EndpointState &msg){
    geometry_msgs::PoseStamped stamp_msg;
    stamp_msg.pose = msg.pose;//現在の位置姿勢を格納
    stamp_msg.header.frame_id="/base";//このPathのローカル座標系
    stamp_msg.header.stamp = ros::Time::now();//この位置を記録した時刻
    pub_lmsg.header = stamp_msg.header;//publish用のPath変数のheaderを設定
    
    pub_lmsg.poses.push_back(stamp_msg);//現在の手先姿勢をpublish用のPath変数に追加
    pub_left_path.publish(pub_lmsg);
}

//右手の位置姿勢が更新されると呼ばれる関数
void right_callback(const baxter_core_msgs::EndpointState &msg){
    geometry_msgs::PoseStamped stamp_msg;
    stamp_msg.pose = msg.pose;//現在の位置姿勢を格納
    stamp_msg.header.frame_id="/base";//このPathのローカル座標系
    stamp_msg.header.stamp = ros::Time::now();//この位置を記録した時刻
    pub_rmsg.header = stamp_msg.header;//publish用のPath変数のheaderを設定
    
    pub_rmsg.poses.push_back(stamp_msg);//現在の手先姿勢をpublish用のPath変数に追加
    pub_right_path.publish(pub_rmsg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "display_test_node");
    cout << "Initializing node... " << endl;
    ros::NodeHandle n;
    
    pub_left_path= n.advertise<nav_msgs::Path>("display_left_topic", 1);//baxterの手先経路を描画するためのnav_msgs::Pathを変数
    pub_right_path= n.advertise<nav_msgs::Path>("display_right_topic", 1);//baxterの手先経路を描画するためのnav_msgs::Pathを変数
    sub_left_path = n.subscribe("/robot/limb/left/endpoint_state", 1, &left_callback);//baxterの左手手先経路
    sub_right_path = n.subscribe("/robot/limb/right/endpoint_state", 1, &right_callback);//baxterの右手手先経路
    ros::spin();
    return 0;
}
