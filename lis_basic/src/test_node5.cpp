//write by C++
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#include <lis_msgs/End_PosesArray.h>
using namespace std;

class Circle_path{
 private:
  ros::NodeHandle n_;
  ros::Publisher pub_topic_;
  lis_msgs::End_PosesArray pub_msg_;//publishする手先姿勢データ
  
 public:
  Circle_path(){
     pub_topic_ = n_.advertise<lis_msgs::End_PosesArray>("hand_pathArray_topic", 10);
  }

  void gen_path(){
    //tf関係の変数
    tf::TransformListener listener;
    geometry_msgs::PoseStamped stamped_in, stamped_out;
    ros::Rate loop_rate(0.3);//0.3Hzでfor文を回してデータをpublishする

    for(int i = 0; i < 5; i++) {
      //tf::Stamped<tf::Transform>型の構造体にdesk座標系でみた目標左手先位置の設定をし、geometry_msgs::PoseStamped構造体に変換
      tf::poseStampedTFToMsg (tf::Stamped<tf::Transform>(tf::Transform(tf::Quaternion(0.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0), tf::Vector3(-0.05*i, -0.1, 0.0)), ros::Time::now(), "/desk"), stamped_in);
      
      //geometry_msgs::PoseStamped構造体stamped_inを/baseからみたデータに変換 
      listener.waitForTransform("/base", ros::Time(0), "/desk", stamped_in.header.stamp, "/desk", ros::Duration(1.0));//http://wiki.ros.org/ja/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29#lookupTransform.2BMG4wiDCKdnpcVXaEMGo-API
      listener.transformPose ("/base", ros::Time(0), stamped_in,"/desk", stamped_out);//http://mirror.umd.edu/roswiki/doc/groovy/api/tf/html/c++/classtf_1_1TransformListener.html

      //baseからみた目標左手先データをpublish用構造体に格納
      pub_msg_.left.push_back(stamped_out.pose);
    }
    
    pub_msg_.header = stamped_out.header;
    pub_topic_.publish(pub_msg_);
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
