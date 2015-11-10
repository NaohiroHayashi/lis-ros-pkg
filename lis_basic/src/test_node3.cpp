//write by C
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#include <lis_msgs/End_poses.h>
using namespace std;

ros::Publisher pub_topic;
lis_msgs::End_poses hand_poses;//publishする手先姿勢データ

void gen_path(){
        //tf関係の変数
        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::StampedTransform tf_left_pose;
        tf::TransformListener listener;
        ros::Rate loop_rate(0.3);//0.3Hzでfor文を回してデータをpublishする
        
        for(int i=0; i<5; i++){
            //現在時刻をros::Time型の構造体に保存
            ros::Time current_time = ros::Time::now();
            
            //~ //tf::broadcast書き方2 まとめて１行で書いてしまう場合
            br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0), tf::Vector3(0.0, 0.0+0.01*i, -0.2)), current_time, "/desk", "/left_target"));

            try{
                  listener.waitForTransform("/base", ros::Time(0), "/left_target",  current_time, "/desk", ros::Duration(1.0));//current_time時刻の/baseから見た/left_targetが登録される間で待つ
                  listener.lookupTransform("/base", ros::Time(0), "/left_target", current_time, "/desk", tf_left_pose); //current_time時刻の/baseから見た/left_targetの姿勢を取得
             }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
            }
            
            //tfからgeometryへの変換　書き方その1　ｔｆの変換関数
            tf::poseTFToMsg (tf_left_pose, hand_poses.left);

            hand_poses.header.frame_id = tf_left_pose.frame_id_;//このデータが、どの座標系から見たものか代入 ここでは/base
            hand_poses.header.stamp = tf_left_pose.stamp_;//このデータがいつの時刻のものか代入
            
            pub_topic.publish(hand_poses);
            loop_rate.sleep();
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_path_node");
    ros::NodeHandle n;

    cout << "Initializing node... " << endl;

    pub_topic = n.advertise<lis_msgs::End_poses>("hand_path_topic", 10);
    
    //決め打ちの右手の姿勢
    hand_poses.right.position.x = 0.656982770038;
    hand_poses.right.position.y = -0.852598021641;
    hand_poses.right.position.z = 0.0388609422173;
    hand_poses.right.orientation.x = 0.367048116303;
    hand_poses.right.orientation.y = 0.885911751787;
    hand_poses.right.orientation.z = -0.108908281936;
    hand_poses.right.orientation.w = 0.261868353356;
       
    gen_path();

    return 0;
}
