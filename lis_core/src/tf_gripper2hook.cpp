// Copyright 2015 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_gripper2hook");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    tf::Transform rtransform,ltransform;
    tf::Quaternion q;

    q.setRPY(0, 0, 0);
    rtransform.setRotation(q);
    //~ rtransform.setOrigin( tf::Vector3(-0.09, 0.0, 0.115) );
    //~ rtransform.setOrigin( tf::Vector3(-0.09, 0.0, 0.12) );
    rtransform.setOrigin( tf::Vector3(0.0, 0.0, 0.12) );
    q.setRPY(0, 0, 0);
    ltransform.setRotation(q);
    //~ ltransform.setOrigin( tf::Vector3(-0.09, 0.0, 0.115) );
    //~ ltransform.setOrigin( tf::Vector3(-0.09, 0.0, 0.12) );
    ltransform.setOrigin( tf::Vector3(0.0, 0.0, 0.12) );
    ros::Rate rate(10.0);
    
    while (n.ok()){
        br.sendTransform(tf::StampedTransform(rtransform, ros::Time::now(), "right_gripper", "right_endgripper2"));
        br.sendTransform(tf::StampedTransform(ltransform, ros::Time::now(), "left_gripper", "left_endgripper2"));
        rate.sleep();
    }
    return 0;
};
