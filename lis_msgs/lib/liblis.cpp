#include <lis_msgs/lis.h>
using namespace std;
using namespace cv;
using namespace object_manipulator;
using namespace msg;

//tf型のStampedTransform をgeometry型Poseに変換して返す
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

//tf型のquaternionをgeometry型に変換して返す
geometry_msgs::Quaternion q_tf2gm(tf::Quaternion tq) {
    geometry_msgs::Quaternion gq;
    gq.x = tq.x();
    gq.y = tq.y();
    gq.z = tq.z();
    gq.w = tq.w();
    return gq;
}

//Vector3同士の外積を計算して返す
geometry_msgs::Vector3 cal_cross(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b){
    geometry_msgs::Vector3 c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

//Vector3同士の内積を計算して返す
double cal_inner(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

//Calculate a distance betwen p1 and p2
double cal_p2p_dist(geometry_msgs::Point p1, geometry_msgs::Point p2){
    double x = p1.x - p2.x;
    double y = p1.y - p2.y;
    double z = p1.z - p2.z;
    return sqrt(x*x+y*y+z*z);
}
//クォータニオンを回転行列に変換、標準基底のベクトルを返す
geometry_msgs::Vector3 cal_axis_vector(geometry_msgs::Quaternion q, char axis){
    geometry_msgs::Vector3 e;
    
    switch(axis){
        case 'x':
            e.x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            e.y = 2.0 * (q.x * q.y + q.z * q.w);
            e.z = 2.0 * (q.z * q.x - q.y * q.w);
            break;
        case 'y':
            e.x = 2.0 * q.x * q.y - 2.0 * q.w * q.z;
            e.y = 1.0 - 2.0 * q.x * q.x - 2.0 * q.z * q.z;
            e.z = 2.0 * q.y * q.z + 2.0 * q.w * q.x;
            break;
        case 'z':
            e.x = 2.0f * q.x * q.z + 2.0f * q.w * q.y;
            e.y = 2.0f * q.y * q.z - 2.0f * q.w * q.x;
            e.z = 1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y;
            break;
        default:
            cout << "error parameter! function cal_axis_vector requires only 'x' or 'y' or 'z'" << endl;
            break;
    }
    
    return e;
}

//正規化したベクトルを返す
geometry_msgs::Vector3 normalization_vector(geometry_msgs::Vector3 v){
    geometry_msgs::Vector3 msg;
    float n = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    msg.x = v.x/n;
    msg.y = v.y/n;
    msg.z = v.z/n;
    
    return msg;
}
//Calcurate deg between x-axes
double cal_x2x_orientation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2){
    geometry_msgs::Vector3 v1 = cal_axis_vector(q1,'x');
    geometry_msgs::Vector3 v2 = cal_axis_vector(q2,'x');
    return fabs(acos(cal_inner(v1,v2)/(sqrt(v1.x*v1.x+v1.y*v1.y+v1.z*v1.z)*sqrt(v2.x*v2.x+v2.y*v2.y+v2.z*v2.z)))*180.0/M_PI);
}

//Calcurate deg between z-axes
double cal_z2z_orientation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2){
    geometry_msgs::Vector3 v1 = cal_axis_vector(q1,'z');
    geometry_msgs::Vector3 v2 = cal_axis_vector(q2,'z');
    return fabs(acos(cal_inner(v1,v2)/(sqrt(v1.x*v1.x+v1.y*v1.y+v1.z*v1.z)*sqrt(v2.x*v2.x+v2.y*v2.y+v2.z*v2.z)))*180.0/M_PI);
}

//二つの点のユークリッド距離を計算
double cal_length_coordinate(geometry_msgs::Point p1, geometry_msgs::Point p2){
    double x = p1.x - p2.x;
    double y = p1.y - p2.y;
    double z = p1.z - p2.z;
    return sqrt(x*x + y*y + z*z);
}

//回転行列(<cv::Mat)をgeometry_msgs::Quaternionに変換して返す
geometry_msgs::Quaternion MatrixToQuaternion(cv::Mat R){
    geometry_msgs::Quaternion q;
    MATRIX mat;
    mat._11 = R.at<double>(0,0); mat._12 = R.at<double>(0,1); mat._13 = R.at<double>(0,2);
    mat._21 = R.at<double>(1,0); mat._22 = R.at<double>(1,1); mat._23 = R.at<double>(1,2);
    mat._31 = R.at<double>(2,0); mat._32 = R.at<double>(2,1); mat._33 = R.at<double>(2,2);
    double s;
    double tr = mat._11 + mat._22 + mat._33 + 1.0f;
    if (tr >= 1.0f) {
        s = 0.5f / sqrt(tr);
      q.w = 0.25f / s;
      q.x= (mat._23 - mat._32) * s;
            q.y= (mat._31 - mat._13) * s;
      q.z= (mat._12 - mat._21) * s;
      return q;
        }else{
            double max;
      if(mat._22 > mat._33){
       max = mat._22;
      }else{
       max = mat._33;
      }
       
        if (max < mat._11) {
            s = sqrt(mat._11 - (mat._22 + mat._33) + 1.0f);
            double x = s * 0.5f;
            s = 0.5f / s;
            q.x= x;
            q.y= (mat._12 + mat._21) * s;
            q.z= (mat._31 + mat._13) * s;
            q.w= (mat._23 - mat._32) * s;
            return q;
        }else if (max == mat._22) {
            s = sqrt(mat._22 - (mat._33 + mat._11) + 1.0f);
            double y = s * 0.5f;
            s = 0.5f / s;
            q.x= (mat._12 + mat._21) * s;
            q.y= y;
            q.z= (mat._23 + mat._32) * s;
            q.w= (mat._31 - mat._13) * s;
            return q;
        }else{
            s = sqrt(mat._33 - (mat._11 + mat._22) + 1.0f);
            double z = s * 0.5f;
            s = 0.5f / s;
            q.x= (mat._31 + mat._13) * s;
            q.y= (mat._23 + mat._32) * s;
            q.z= z;
            q.w= (mat._12 - mat._21) * s;
            return q;
        }
    }
}

//回転行列(<geometry_msgs::Vector3> mx,my,mz)をgeometry_msgs::Quaternionに変換して返す
geometry_msgs::Quaternion matrix2quaternion(geometry_msgs::Vector3 m_x, geometry_msgs::Vector3 m_y, geometry_msgs::Vector3 m_z){
    geometry_msgs::Quaternion q;
    MATRIX mat;
    mat._11 = m_x.x; mat._12 = m_x.y; mat._13 = m_x.z;
    mat._21 = m_y.x; mat._22 = m_y.y; mat._23 = m_y.z;
    mat._31 = m_z.x; mat._32 = m_z.y; mat._33 = m_z.z;
    double s;
    double tr = mat._11 + mat._22 + mat._33 + 1.0f;
    if (tr >= 1.0f) {
        s = 0.5f / sqrt(tr);
      q.w = 0.25f / s;
      q.x= (mat._23 - mat._32) * s;
            q.y= (mat._31 - mat._13) * s;
      q.z= (mat._12 - mat._21) * s;
      return q;
        }else{
            double max;
      if(mat._22 > mat._33){
       max = mat._22;
      }else{
       max = mat._33;
      }
       
        if (max < mat._11) {
            s = sqrt(mat._11 - (mat._22 + mat._33) + 1.0f);
            double x = s * 0.5f;
            s = 0.5f / s;
   q.x= x;
   q.y= (mat._12 + mat._21) * s;
   q.z= (mat._31 + mat._13) * s;
   q.w= (mat._23 - mat._32) * s;
      return q;
        }else if (max == mat._22) {
            s = sqrt(mat._22 - (mat._33 + mat._11) + 1.0f);
            double y = s * 0.5f;
            s = 0.5f / s;
   q.x= (mat._12 + mat._21) * s;
            q.y= y;
   q.z= (mat._23 + mat._32) * s;
   q.w= (mat._31 - mat._13) * s;
      return q;
        }else{
            s = sqrt(mat._33 - (mat._11 + mat._22) + 1.0f);
            double z = s * 0.5f;
            s = 0.5f / s;
   q.x= (mat._31 + mat._13) * s;
   q.y= (mat._23 + mat._32) * s;
            q.z= z;
   q.w= (mat._12 - mat._21) * s;
      return q;
        }
    }
}

//球面線形補間 http://www21.atwiki.jp/opengl/pages/149.html
geometry_msgs::Quaternion Spherical_Linear_Interpolation( geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2, double t){
    geometry_msgs::Quaternion q, iq2;
    double qr = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    double ss = 1.0f - qr * qr;

    if (ss == 0.0f) {
        q.w = q1.w;
        q.x = q1.x;
        q.y = q1.y;
        q.z = q1.z;
        return q;
    }
    else if(qr<0) {
        iq2.w = -q2.w;
        iq2.x = -q2.x;
        iq2.y = -q2.y;
        iq2.z = -q2.z;
        return Spherical_Linear_Interpolation(q1,iq2,t);
    }
    else {
        double sp = sqrt(ss);
        double ph = acos(qr);
        double pt = ph * t;
        double t1 = sin(pt) / sp;
        double t0 = sin(ph - pt) / sp;

        q.w = q1.w * t0 + q2.w * t1;
        q.x = q1.x * t0 + q2.x * t1;
        q.y = q1.y * t0 + q2.y * t1;
        q.z = q1.z * t0 + q2.z * t1;
        return q;
    }
}

void gauss(double a[3][4],double xx[3])
{
        int n=3;
        int N=3;
        int i,j,k,l,pivot;
        double x[N];
        double p,q,m,b[1][N+1];

        for(i=0;i<N;i++) {
                m=0;
                pivot=i;

                for(l=i;l<N;l++) {
                        if(fabs(a[l][i])>m) {   //i列の中で一番値が大きい行を選ぶ
                                m=fabs(a[l][i]);
                                pivot=l;
                        }
                }

                if(pivot!=i) {                          //pivotがiと違えば、行の入れ替え
                        for(j=0;j<N+1;j++) {
                                b[0][j]=a[i][j];        
                                a[i][j]=a[pivot][j];
                                a[pivot][j]=b[0][j];
                        }
                }
        }

        for(k=0;k<N;k++) {
                p=a[k][k];              //対角要素を保存
                a[k][k]=1;              //対角要素は１になることがわかっているから

                for(j=k+1;j<N+1;j++) {
                        a[k][j]/=p;
                }

                for(i=k+1;i<N;i++) {
                        q=a[i][k];

                        for(j=k+1;j<N+1;j++) {
                                a[i][j]-=q*a[k][j];
                        }
                a[i][k]=0;              //０となることがわかっているところ
                }
        }

//解の計算
        for(i=N-1;i>=0;i--) {
                x[i]=a[i][N];
                for(j=N-1;j>i;j--) {
                        x[i]-=a[i][j]*x[j];
                }
        }

//行列が最後どうなったか見たいときに実行
//~ #if CHECK==1
        //~ for(i=0;i<N;i++) {
                //~ for(j=0;j<N+1;j++) {
                        //~ printf("%10.3f",a[i][j]);
                //~ }
                //~ printf("\n");
                //~ 
        //~ }
//~ #endif
//~ 
        //~ printf("解は\n");
        for(i=0;i<N;i++) {
                //~ printf("%f\n",x[i]);
                xx[i]=x[i];
        }

}

/*************************************/
/* 2線分の交点                       */
/*      A + r(B - A) と C + s(D - C) */
/*      P : 交点の座標               */
/*      return : =-1 : 交点が無い    */
/*               =0 : 交点が線分の外 */
/*               =1 : 交点がある     */
/*************************************/
//https://www.sist.ac.jp/~suganuma/cpp/2-bu/7-sho/C++/basic.htm#cross_line_s
int IntersectSegements(const lis_msgs::Point2D &A, const lis_msgs::Point2D &B, 
  const lis_msgs::Point2D &C, const lis_msgs::Point2D &D, lis_msgs::Point2D &P)
{
  double r, s, BUNBO, EPS = 1.0e-8;
 lis_msgs::Point2D AC;
  int sw = -1;

  AC.x = C.x - A.x;
  AC.y = C.y - A.y;
  BUNBO = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);
  if (fabs(BUNBO) > EPS) {
    r = ((D.y - C.y) * AC.x - (D.x - C.x) * AC.y) / BUNBO;
    s = ((B.y - A.y) * AC.x - (B.x - A.x) * AC.y) / BUNBO;
    if (r > -EPS && r < 1.0+EPS && s > -EPS && s < 1.0+EPS) {
      P.x = A.x + r * (B.x - A.x);
      P.y = A.y + r * (B.y - A.y);
      sw   = 1;
    }
    else
      sw = 0;
  }

  return sw;
}

lis_msgs::Plane CalPlane(const geometry_msgs::Point &A, const geometry_msgs::Point &B, const geometry_msgs::Point &C)
{
  geometry_msgs::Vector3 vector12 = object_manipulator::msg::createVector3Msg(B.x - A.x, B.y - A.y, B.z - A.z);
  geometry_msgs::Vector3 vector13 = object_manipulator::msg::createVector3Msg(C.x - A.x, C.y - A.y, C.z - A.z);

  //単位ベクトルの法線ベクトルを計算する
  geometry_msgs::Vector3 unit_normal_vector = normalization_vector(cal_cross(vector12, vector13));
  
  double d = -(unit_normal_vector.x * A.x + unit_normal_vector.y * A.y + unit_normal_vector.z * A.z);
  
  lis_msgs::Plane plane;
  plane.a = unit_normal_vector.x;
  plane.b = unit_normal_vector.y;
  plane.c = unit_normal_vector.z;
  plane.d = d;
  //平面の方程式の係数a, b, c, dを返す
  return plane;
}

//http://www.sousakuba.com/Programming/gs_plane_line_intersect.html
bool IntersectPlaneAndLine(
  const geometry_msgs::Point &A,   //線分始点
  const geometry_msgs::Point &B,   //線分終点
  const lis_msgs::Plane &PL, //平面
  geometry_msgs::Point &out)//戻り値　交点が見つかれば格納される)
{ 
  //平面上の点P
  geometry_msgs::Point P = object_manipulator::msg::createPointMsg(PL.a * PL.d, PL.b * PL.d, PL.c * PL.d );

  //PA PBベクトル
  geometry_msgs::Vector3 PA = object_manipulator::msg::createVector3Msg(P.x - A.x, P.y - A.y, P.z - A.z );
  geometry_msgs::Vector3 PB = object_manipulator::msg::createVector3Msg(P.x - B.x, P.y - B.y, P.z - B.z );

  //PA PBそれぞれを平面法線と内積
  double dot_PA = PA.x * PL.a + PA.y * PL.b + PA.z * PL.c;
  double dot_PB = PB.x * PL.a + PB.y * PL.b + PB.z * PL.c;

  //これは線端が平面上にあった時の計算の誤差を吸収しています。調整して使ってください。
  if ( abs(dot_PA) < 0.000001 ) { dot_PA = 0.0; } 
  if ( abs(dot_PB) < 0.000001 ) { dot_PB = 0.0; }

  //交差判定
  if( dot_PA == 0.0 && dot_PB == 0.0 ) {
    //両端が平面上にあり、交点を計算できない。
    return false;
  } else
  if ( ( dot_PA >= 0.0 && dot_PB <= 0.0 ) ||
       ( dot_PA <= 0.0 && dot_PB >= 0.0 ) ) {
     //内積の片方がプラスで片方がマイナスなので、交差している
     
  } else {
    //交差していない
    return false;
  }

  //以下、交点を求める 

  geometry_msgs::Vector3 AB = object_manipulator::msg::createVector3Msg(B.x - A.x, B.y - A.y, B.z - A.z );

  //交点とAの距離 : 交点とBの距離 = dot_PA : dot_PB
  double hiritu = abs(dot_PA) / ( abs(dot_PA) + abs(dot_PB) );

  out.x = A.x + ( AB.x * hiritu );
  out.y = A.y + ( AB.y * hiritu );
  out.z = A.z + ( AB.z * hiritu );

  return true;
}

