#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <lis_msgs/End_poses.h>
#include <lis_msgs/lis.h>
using namespace std;

ros::Publisher pub_end_pose;
ros::Subscriber sub_target_pose;

void target_pose2end_pose(const lis_msgs::End_poses tp){
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    lis_msgs::End_poses ep;
    tf::StampedTransform rep,lep;
    ros::Time now = ros::Time::now();//このタイムスタンプのend_poseは遅延でまだ、できていない　下の関数で少し待ってから見に行く
    int i =0;
    ros::Rate rate(15);//5hz(0.1s間隔)で送信する

    while (i < 5){//0.2sx10ループ＝2秒間座標を送信し続ける
        //TFにbaseから見たtarget_pose（目標グリッパ位置）の座標系を登録
        br.sendTransform(tf::StampedTransform(tf::Transform(
        tf::Quaternion(tp.left.orientation.x, tp.left.orientation.y, tp.left.orientation.z, tp.left.orientation.w),
        tf::Vector3(tp.left.position.x, tp.left.position.y, tp.left.position.z)
        ), now, "base", "left_target_pose"));
        
        br.sendTransform(tf::StampedTransform(tf::Transform(
        tf::Quaternion(tp.right.orientation.x, tp.right.orientation.y, tp.right.orientation.z, tp.right.orientation.w),
        tf::Vector3(tp.right.position.x, tp.right.position.y, tp.right.position.z)
        ), now, "base", "right_target_pose"));
        
        //TFにtarget_poseから見たend_pose（エンドステイツ）の座標系を登録
        //~ br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0350, 0.000, -0.115-0.13855)), now, "left_target_pose", "left_target_wrist"));
        //~ br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0350, 0.000, -0.115-0.13855)), now, "right_target_pose", "right_target_wrist"));
        br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0, 0.000, -0.12)), now, "left_target_pose", "left_target_wrist"));
        br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0, 0.000, -0.12)), now, "right_target_pose", "right_target_wrist"));
        br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0, 0.000, -0.12)), now, "left_target_pose", "left_target_handend"));
        br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0), tf::Vector3(0.0, 0.000, -0.12)), now, "right_target_pose", "right_target_handend"));
        rate.sleep();
        i++;
    }
    
    //baseから見た[limb]_end_poseの位置姿勢をTFから取得する
    try{
          listener.lookupTransform("base", "left_target_wrist", ros::Time(0), lep);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

    try{
          listener.lookupTransform("base", "right_target_wrist", ros::Time(0), rep);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

    //変換した座標データをpublish用のデータ変数に格納
    ep.left = pose_tf2gm(&lep);
    ep.right = pose_tf2gm(&rep);
    
    pub_end_pose.publish(ep);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_pose_node");
    ros::NodeHandle n;
    cout << "Initializing node... " << endl;
    lis_msgs::End_poses end_poses;

    pub_end_pose = n.advertise<lis_msgs::End_poses>("end_pose_topic", 10);
    sub_target_pose = n.subscribe("hand_path_topic", 10, &target_pose2end_pose);
    
    ros::spin(); //callback
    return 0;
}
