// display_object.cpp
// Copyright 2014 Naohiro Hayashi <kaminuno@kaminuno-ThinkPad-T440p> 
#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
using namespace std;

class Display_obcect{
    private:
    ros::NodeHandle n;
    ros::Publisher desk_pub;
    //Marker関係
    visualization_msgs::Marker desk;
    
    public:
    Display_obcect(){
        //path_publish
        desk_pub = n.advertise<visualization_msgs::Marker>("display_object_desk", 1);

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        desk.header.frame_id = "box";
        desk.header.stamp = ros::Time::now();

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        desk.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD and DELETE
        desk.action = visualization_msgs::Marker::ADD;

         // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        desk.pose.position.x = 0;
        desk.pose.position.y = 0.0;
        desk.pose.position.z = 0.0;
        desk.pose.orientation.x = 0.0;
        desk.pose.orientation.y = 0.0;
        desk.pose.orientation.z = 0.0;
        desk.pose.orientation.w = 1.0;
        
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        desk.scale.x = 0.3;
        desk.scale.z = 0.2;
        desk.scale.y = 0.3;

        // Set the color -- be sure to set alpha to something non-zero!
        desk.color.r = 0.3f;
        desk.color.g = 0.3f;
        desk.color.b = 0.0f;
        desk.color.a = 0.5;
        desk.lifetime = ros::Duration();

    }
    
    void show(){
        ros::Rate r(1);
        while (ros::ok())
        {
            desk_pub.publish(desk);// Publish the marker
            r.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_object_node");
    cout << "Initializing node... " << endl;

    Display_obcect display_obcect;
    display_obcect.show();
    ros::spin();
    return 0;
}
