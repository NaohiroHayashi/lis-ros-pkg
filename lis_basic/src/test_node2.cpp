//write by C++
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#include <lis_msgs/End_poses.h>
using namespace std;

class Circle_path{
    private:
    ros::NodeHandle n;
    ros::Publisher pub_topic;
    
    lis_msgs::End_poses hand_poses;//publishする手先姿勢データ
    
    public:
    Circle_path(){
       pub_topic = n.advertise<lis_msgs::End_poses>("hand_path_topic", 10);
      
      //決め打ちの右手の姿勢
       hand_poses.right.position.x = 0.656982770038;
       hand_poses.right.position.y = -0.852598021641;
       hand_poses.right.position.z = 0.0388609422173;
       hand_poses.right.orientation.x = 0.367048116303;
       hand_poses.right.orientation.y = 0.885911751787;
       hand_poses.right.orientation.z = -0.108908281936;
       hand_poses.right.orientation.w = 0.261868353356;
    }

    void gen_path(){
            //tf関係の変数
            tf::TransformListener listener;
            geometry_msgs::PoseStamped stamped_in, stamped_out;
            ros::Rate loop_rate(0.3);//0.3Hzでfor文を回してデータをpublishする

            for(int i=0; i<5; i++){
                //tf::Stamped<tf::Transform>型の構造体にdesk座標系でみた目標左手先位置の設定をし、geometry_msgs::PoseStamped構造体に変換
                tf::poseStampedTFToMsg (tf::Stamped<tf::Transform>(tf::Transform(tf::Quaternion(0.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0), tf::Vector3(0.0, 0.0+0.01*i, -0.2)), ros::Time::now(), "/desk"), stamped_in);
                
                //geometry_msgs::PoseStamped構造体stamped_inを/baseからみたデータに変換 
                listener.waitForTransform("/base", ros::Time(0), "/desk", stamped_in.header.stamp, "/desk", ros::Duration(1.0));//http://wiki.ros.org/ja/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29#lookupTransform.2BMG4wiDCKdnpcVXaEMGo-API
                listener.transformPose ("/base", ros::Time(0), stamped_in,"/desk", stamped_out);//http://mirror.umd.edu/roswiki/doc/groovy/api/tf/html/c++/classtf_1_1TransformListener.html

                //baseからみた目標左手先データをpublish用構造体に格納
                hand_poses.left = stamped_out.pose;
                hand_poses.header = stamped_out.header;
                
                pub_topic.publish(hand_poses);
                loop_rate.sleep();
            }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_path_node");

    cout << "Initializing node... " << endl;
    Circle_path circle_path;
    circle_path.gen_path();

    return 0;
}
