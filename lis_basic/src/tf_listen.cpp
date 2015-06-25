#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
using namespace std;

//おまけ　tf::StampedTransform型をgeometry_msgs::Pose型に変換する関数
geometry_msgs::Pose pose_tf2gm(tf::StampedTransform *tf_pose) {
    geometry_msgs::Pose gm_pose;
    gm_pose.position.x = tf_pose->getOrigin().x();
    gm_pose.position.y = tf_pose->getOrigin().y();
    gm_pose.position.z = tf_pose->getOrigin().z();
    gm_pose.orientation.x = tf_pose->getRotation().x();
    gm_pose.orientation.y = tf_pose->getRotation().y();
    gm_pose.orientation.z = tf_pose->getRotation().z();
    gm_pose.orientation.w = tf_pose->getRotation().w();
    return gm_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_listen");
    ros::NodeHandle n;
    cout << "Initializing node... " << endl;
    
    tf::TransformListener listener;
    ros::Time now = ros::Time::now();//このタイムスタンプのend_poseは遅延でまだ、できていない　下の関数で少し待ってから見に行く
    ros::Rate rate(1);//1hz(1s間隔)
    tf::StampedTransform tf_pose;
    
    while (n.ok()){
        try{
          listener.lookupTransform("base", "desk", ros::Time(0), tf_pose);       //baseから見た位置姿勢をTFから取得する　ros::Time(0)は最新のタイムスタンプ
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        cout << "並進 " << tf_pose.getOrigin().x() << " " << tf_pose.getOrigin().y() << " " << tf_pose.getOrigin().z() << endl;//得られた並進成分
        cout << "回転 " << tf_pose.getRotation().x() << " " << tf_pose.getRotation().y() << " " << tf_pose.getRotation().z() << " " << tf_pose.getRotation().w() <<  endl;//得られた回転成分(quarternion表記)
        rate.sleep();
    }

    return 0;
}
