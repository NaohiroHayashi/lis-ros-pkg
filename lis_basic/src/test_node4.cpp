#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <tf/transform_broadcaster.h>
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
            ros::Rate loop_rate(0.3);//0.3Hzでfor文を回してデータをpublishする

            for(int i=0; i<5; i++){
                //baseから見た左手の位置
                hand_poses.left.position.x = 0.5;
                hand_poses.left.position.y = 0.2 + 0.05 * i;//loopごとにY軸方向に移動
                hand_poses.left.position.z = 0.2;

                //baseから見た左手の姿勢
                tf::Quaternion q;
                q.setRPY(180.0*M_PI/180.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0);// ロー、ピッチ、ヨーで値をセット http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Quaternion.html#ab34dfef6a01c1ece07bea92d485b0576
                //クォータ二オンの値をmsgに代入
                hand_poses.left.orientation.x = q.x();
                hand_poses.left.orientation.y = q.y();
                hand_poses.left.orientation.z = q.z();
                hand_poses.left.orientation.w = q.w();

                //headerの設定
                hand_poses.header.frame_id = "/base";
                hand_poses.header.stamp = ros::Time::now();

                pub_topic.publish(hand_poses);
                loop_rate.sleep();
            }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");

    cout << "Initializing node... " << endl;
    Circle_path circle_path;
    circle_path.gen_path();

    return 0;
}
