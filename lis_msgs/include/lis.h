#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <lis_msgs/End_PosesArray.h>
#include <lis_msgs/Point2D.h>
#include <lis_msgs/Plane.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <msg_helpers.h>

struct MATRIX {
   double _11, _12, _13;
   double _21, _22, _23;
   double _31, _32, _33;
};
//
//tf::Pose型をgeometry_msgs::Pose型に変換して返す
geometry_msgs::Pose pose_tf2gm(tf::StampedTransform *tf_pose);

//tf::Quaternion型をgeometry_msgs::Quaternion型に変換して返す
geometry_msgs::Quaternion q_tf2gm(tf::Quaternion tq);

//Vector3同士の外積を計算して返す
geometry_msgs::Vector3 cal_cross(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);

//Vector3同士の内積を計算して返す
double cal_inner(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);

//Calculate a distance between geometry_msgs::Point p1 and p2
double cal_p2p_dist(geometry_msgs::Point p1, geometry_msgs::Point p2);

//クォータニオンを回転行列に変換、標準基底のベクトルを返す
geometry_msgs::Vector3 cal_axis_vector(geometry_msgs::Quaternion q, char axis);

//正規化したベクトルを返す
geometry_msgs::Vector3 normalization_vector(geometry_msgs::Vector3 v);

//Calcurate deg between x-axes
double cal_x2x_orientation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

//Calcurate deg between z-axes
double cal_z2z_orientation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

//現在と一つ前の座標系Tのユークリッド距離を計算
double cal_length_coordinate(geometry_msgs::Point p1, geometry_msgs::Point p2);

//回転行列(<cv::Mat)をgeometry_msgs::Quaternionに変換して返す
geometry_msgs::Quaternion MatrixToQuaternion(cv::Mat R);

//回転行列(<geometry_msgs::Vector3> mx,my,mz)をgeometry_msgs::Quaternionに変換して返す
geometry_msgs::Quaternion matrix2quaternion(geometry_msgs::Vector3 m_x, geometry_msgs::Vector3 m_y, geometry_msgs::Vector3 m_z);

//球面線形補間 http://www21.atwiki.jp/opengl/pages/149.html
geometry_msgs::Quaternion Spherical_Linear_Interpolation( geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2, double t);

void gauss(double a[3][4],double xx[3]);

int IntersectSegements(const lis_msgs::Point2D &A, const lis_msgs::Point2D &B, const lis_msgs::Point2D &C, const lis_msgs::Point2D &D, lis_msgs::Point2D &P);

lis_msgs::Plane CalPlane(const geometry_msgs::Point &A, const geometry_msgs::Point &B, const geometry_msgs::Point &C);

bool IntersectPlaneAndLine(
  const geometry_msgs::Point &A,   //線分始点
  const geometry_msgs::Point &B,   //線分終点
  const lis_msgs::Plane &PL, //平面
  geometry_msgs::Point &out);//戻り値　交点が見つかれば格納される)
  
  // rotation n-deg via hand_pose
//~ geometry_msgs::Quaternion move_rotation(geometry_msgs::Pose pose, double deg);
//~ // rotation n-deg via hand_pose
//~ geometry_msgs::Quaternion move_rotation2(geometry_msgs::Pose pose, double deg);
//~ 
//~ geometry_msgs::Point move_position(geometry_msgs::Pose pose, double x, double y, double z);
