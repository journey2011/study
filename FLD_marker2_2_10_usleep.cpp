//rosのヘッダ
//#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <Eigen/Dense>//Eigrn用
#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include <time.h>//処理の時間を出力する
#include <sys/time.h>//C言語の時間取得用
#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include <nav_msgs/Path.h>//経路情報を記録する
#include <geometry_msgs/Twist.h>//ロボットの指令値(速度)用ヘッダー
#include <nav_msgs/Odometry.h>//オドメトリーデータ用
#include<fstream>//ファイル出力用
#include <stdlib.h>//abs関数取得

ros::Subscriber sub;//データをsubcribeする奴
ros::Subscriber odom_sub;//ロボットオドメトリー受信用
ros::Publisher  pub;//速度送信用
ros::Publisher Des_pub_plan;//カメラ経路送信用(指令値)
ros::Publisher Act_pub_plan;//カメラ経路送信用(観測値)
ros::Publisher Est_pub_plan;//カメラ経路送信用(推定値)
std::string source_frame = "map";//mapフレーム

std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_fld = "fld";//FLDの線を表示
std::string win_fld_ty = "FLD_TY";//抽出したFLDのタテ、ヨコの線を表示
std::string win_fld_t = "FLD_T";//抽出したFLDのタテ線を表示
std::string win_fld_y = "FLD_Y";//抽出したFLDのヨコの線を表示

std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_line3 = "line3";//分類分けしたタテ、ヨコ、ナナメの線を表示
std::string win_line4 = "line4";//分類分けしたタテ、ヨコの線を表示

std::string win_dstP = "dstP";//特徴点表示+src画像
std::string win_prev = "prev";//一つ前の画像
std::string win_curr = "curr";//今の画像
std::string win_dst2 = "dst2";//オプティカルフローの動き画像
//std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//カメラ画像
std::string win_graph = "graph";//グラフ作成
std::string win_tate = "tate";//縦線関連の画像表示用


using namespace std;
using namespace cv;
using Eigen::MatrixXd;

//初期定義--------------------------------------------------------------------------------------------------------
int linesu=400;
int kaisu=0,kaisuM1=0,kaisuV1=0;
// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)出を行うように、初期値を TRUE にする
bool reset = true;
bool yoko_histgram = false;//ヨコ線のヒストグラム(true実行)
bool tate_histgram = false;//タテ線のヒストグラム(true実行)
bool time0 = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool X_25=false;
bool TH_90=false;
bool Y_05=false;
bool Y_051=false;
bool Y_052=false;


bool Tracking=false;

double VX,omegaZ,LX,THZ,LY;//ロボットの指令値パラメータ
int act_v=0;//ロボット速度制御用

//時間取得用
ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
struct timeval startTime2, endTime2;  // 構造体宣言
struct timeval startTimeV1, endTimeV1;  // 構造体宣言
struct timeval startTimeM1, endTimeM1;  // 構造体宣言
float realsec,ALLrealsec;//サンプリング時間（C++)
float realsecV1,ALLrealsecV1;//直進動作時間（C++)
float realsecM1,ALLrealsecM1;//回転動作時間（C++)

int template_size=10;
double cropx=10*2.5;//予測範囲の範囲(template_size*n)
double cropy=10*2.5;//予測範囲の範囲(template_size*n)
//double cropx=10*2.5;//予測範囲の範囲(template_size*n)
//double cropy=10*2.5;//予測範囲の範囲(template_size*n)


cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_1;
cv::Mat img_FLD_TY,img_FLD_T,img_FLD_Y,img_graph;
cv::Mat img_tate;//縦線関連の画像表示
double Average_tate_theta,Average_tate_theta_Y;//最大クラスタ内の平均角度を求める

//マーカー関連
cv::Mat_<float>MarkerW[40];//マーカーの世界座標
int ALLMarker=40;//全マーカー個数
float MC_point_prve[50][4];//一つ前のカメラ座標
float pixel[50][2],depth[100],MC_point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換
double LM[1][2],CameraLM[50][2];//ランドマーク世界座標
cv::Mat Zu[40];//センサーの観測値(仮)

//縦線のテンプレート取得関連
vector<cv::Point2f> tate_point_curr,tate_point_prev;//縦線の中点座標(画像座標系)
double DTPC[100],DTPP[100];//depth_tate_point_curr=DTPC,depth_tate_point_prev=DTPP(Depth取得可能縦線中点の数)
int DTPC_ok,DTPP_ok;//(Depth取得可能縦線数)
vector<cv::Point3f> TPCC,TPCP;//tate_point_camera_curr=TPCC,tate_point_camera_prev=TPCP(縦線中点のカメラ座標系)
cv::Mat TPCC_Templ[100],TPCP_Templ[100];//縦線中点のテンプレート画像

//テンプレートマッチ用変数
vector<cv::Point3f> Est_tate_point;//特徴点定義(運動復元
vector<cv::Point2f> Est_tate_pixel;//特徴点定義(運動復元
vector<cv::Point2f> MTPC,MTPP;//テンプレートの中心座標(Matching_Tate_Point_Curr=MTPC,Matching_Tate_Point_Prev=MTPP)
int matchT_curr=0,matchT_prev=0;//マッチングしたテンプレート数
cv::Mat MTTC[500],MTTP[500];//マッチングしたテンプレート画像キープ用(Matching_Tate_Templ_Curr=MTTC,Matching_Tate_Templ_Prev=MTTP)
cv::Mat EST_tate_scope[100];//特徴点周囲の切り取った画像(予測範囲画像)
cv::Mat img_template1,img_master_temp;
cv::Point min_pt1[100], max_pt1[100];//テンプレートマッチング用変数
double min_val1[100], max_val1[100];


cv::Mat MT_curr_Templ[100],MT_prev_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MT_curr_pixel,  MT_prev_pixel;//マッチテンプレートの画像座標系
vector<cv::Point3f> MT_curr_camera, MT_prev_camera;//マッチテンプレートのカメラ座標系
vector<cv::Point3f> MT_curr_world,  MT_prev_world;//マッチテンプレートの世界座標系
vector<cv::Point3f> MT_curr_world2, MT_prev_world2;//マッチテンプレートの世界座標系
double DMT_curr[600],DMT_prev[600];//Depth_Matching_tate_prev(マッチテンプレート座標のDepth)
int DMT_curr_ok=0,DMT_prev_ok;//マッチテンプレートのDepth取得可能数
double length;//テンプレートの距離比較用(追加更新動作)

//テンプレートマッチ用変数(3回目動作用)
cv::Mat MT_curr2_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MT_curr2_pixel;//マッチテンプレートの画像座標系
vector<cv::Point3f> MT_curr2_camera;//マッチテンプレートのカメラ座標系
vector<cv::Point3f> MT_curr2_world;//マッチテンプレートの世界座標
double DMT_curr2[600];//Depth_Matching_pixel_prev(マッチテンプレート座標のDepth)
int DMT_curr2_ok=0;//マッチテンプレートのDepth取得可能数

int times=0;

double MTcoix[300],MTcoiy[300];
vector<cv::Point3f> Est_MT_point;//特徴点定義(運動復元
vector<cv::Point2f> Est_MT_pixel;//特徴点定義(運動復元
cv::Mat EST_MT_scope[100];//予測範囲クロップ
int EST_MT_ok=0;//画面内のテンプレート数
double CameraLMT[600];
cv::Mat Zu_T[600];//センサーの観測値(仮)

geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー

//ロボット動作関連
double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double Act_RobotX=0,Act_RobotY=0,Act_RobotTH=0;//ロボットの状態方程式(実際の状態)
double Des_RobotX=0,Des_RobotY=0,Des_RobotTH=0;//ロボットの状態方程式(理想状態)
double Est_RobotX=0,Est_RobotY=0,Est_RobotTH=0;//ロボットの状態方程式(推定状態)
double Act_RobotV,Des_RobotV;//ロボットの速度ベクトル(実測値,指令値)

//LPF関連
double depth_LPF_pre[100];
double depth_raw_pre[100];
double LPF_f = 0.01;
double LPF_T = 1/(2*M_PI*LPF_f);

//feedback制御関連
double Goal_RobotX,Goal_RobotY;//目的地のxy座標をあらかじめ入力
double distance_to_goal;
double Goal_RobotTH;//目的地のxy座標からロボットと目的地のなす角を計算
bool rotation_goal = false;
bool straight_goal = false;

//subgoal関連
double subgoalX = 0,subgoalY = 0;
ros::Time straight_begin;
bool decide_subgoal = true;

//カルマンフィルタ関連
cv::Mat ACT_Robot,DES_Robot,EST_Robot;//状態方程式の行列化
cv::Mat At,Mt,Ft,Cov,Qt,K,Ht,I,hu;

geometry_msgs::Pose Des_Robot_pose;//ロボットtf(指令値)
geometry_msgs::Pose Act_Robot_pose;//ロボットtf(観測値)
geometry_msgs::Pose Est_Robot_pose;//ロボットtf(推定値)
nav_msgs::Path Des_path;//カメラ経路表示設定
nav_msgs::Path Act_path;//カメラ経路表示設定
nav_msgs::Path Est_path;//カメラ経路表示設定
geometry_msgs::PoseStamped Des_pose;//ロボット姿勢(指令値)
geometry_msgs::PoseStamped Act_pose;//ロボット姿勢(観測値)
geometry_msgs::PoseStamped Est_pose;//ロボット姿勢(推定値)
//--------------------------------------------------------------------------------------------------------------------------

//システム出力データ(ここで出力ファイルの指定をしている)--------------------------------------------------------------------
ofstream est_kaisu("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/est_kaisu.txt");//推定結果取得時間
ofstream est_robotxx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/est_robotxx.txt");//ロボットの推定位置X
ofstream est_robotyy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/est_robotyy.txt");//ロボットの推定位置Y
ofstream est_robotthth("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/est_robotthth.txt");//ロボットの推定姿勢θ
ofstream act_robotxx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/act_robotxx.txt");//ロボットの指令位置X
ofstream act_robotyy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/act_robotyy.txt");//ロボットの指令位置Y
ofstream act_robotthth("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/act_robotthth.txt");//ロボットの指令位置θ

ofstream MP_kaisu("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MP_kaisu.txt");//座標取得時間
ofstream MC_worldx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MC_worldx.txt");//マーカーの世界座標X
ofstream MC_worldy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MC_worldy.txt");//マーカーの世界座標Y
ofstream MP_worldx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MP_worldx.txt");//特徴線の世界座標X
ofstream MP_worldy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MP_worldy.txt");//特徴線の世界座標Y
ofstream MC_camerax("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MC_camerax.txt");//マーカーのカメラ座標X
ofstream MC_cameray("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MC_cameray.txt");//マーカーのカメラ座標Y
ofstream MP_camerax("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MP_camerax.txt");//特徴線のカメラ座標X
ofstream MP_cameray("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/MP_cameray.txt");//特徴線のカメラ座標Y

ofstream kal_cov_time("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_cov_time.txt");//誤差共分散の値分析(取得時間)
ofstream kal_covX1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covX1.txt");//誤差共分散の値分析(Σxx)
ofstream kal_covX2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covX2.txt");//誤差共分散の値分析(Σxy)
ofstream kal_covX3("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covX3.txt");//誤差共分散の値分析(Σxθ)
ofstream kal_covY1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covY1.txt");//誤差共分散の値分析
ofstream kal_covY2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covY2.txt");//誤差共分散の値分析(Σyy)
ofstream kal_covY3("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covY3.txt");//誤差共分散の値分析
ofstream kal_covTH1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covTH1.txt");//誤差共分散の値分析
ofstream kal_covTH2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covTH2.txt");//誤差共分散の値分析
ofstream kal_covTH3("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_covTH3.txt");//誤差共分散の値分析(Σθθ)

ofstream kal_gain_time("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gain_time.txt");//カルマンゲインの値分析(取得時間)
ofstream kal_gainX1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gainX1.txt");//カルマンゲインの値分析(距離成分)
ofstream kal_gainX2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gainX2.txt");//カルマンゲインの値分析(角度成分)
ofstream kal_gainY1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gainY1.txt");//カルマンゲインの値分析(距離成分)
ofstream kal_gainY2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gainY2.txt");//カルマンゲインの値分析(角度成分)
ofstream kal_gainTH1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gainTH1.txt");//カルマンゲインの値分析(距離成分)
ofstream kal_gainTH2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kal_gainTH2.txt");//カルマンゲインの値分析(角度成分)
ofstream kansoku_Zu1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kansoku_Zu1.txt");//観測残差の確認(観測距離)
ofstream kansoku_Zu2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kansoku_Zu2.txt");//観測残差の確認(観測角度)
ofstream kansoku_hu1("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kansoku_hu1.txt");//観測残差の確認(推定距離)
ofstream kansoku_hu2("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/kansoku_hu2.txt");//観測残差の確認(推定角度)


ofstream Test_MT_kaisu("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_kaisu.txt");//失敗対策確認用(取得時間)
ofstream Test_MT_C2_Cx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Cx.txt");//失敗対策確認用
ofstream Test_MT_C2_Cy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Cy.txt");//失敗対策確認用
ofstream Test_MT_C1_Cx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Cx.txt");//失敗対策確認用
ofstream Test_MT_C1_Cy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Cy.txt");//失敗対策確認用
ofstream Test_MT_C2_Wx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Wx.txt");//失敗対策確認用
ofstream Test_MT_C2_Wy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Wy.txt");//失敗対策確認用
ofstream Test_MT_C1_Wx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Wx.txt");//失敗対策確認用
ofstream Test_MT_C1_Wy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Wy.txt");//失敗対策確認用

ofstream Test0_MT_kaisu("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_kaisu.txt");//失敗対策確認用(取得時間)
ofstream Test0_MT_C2_Cx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C2_Cx.txt");//失敗対策確認用
ofstream Test0_MT_C2_Cy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C2_Cy.txt");//失敗対策確認用
ofstream Test0_MT_C1_Cx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C1_Cx.txt");//失敗対策確認用
ofstream Test0_MT_C1_Cy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C1_Cy.txt");//失敗対策確認用
ofstream Test0_MT_C2_Wx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C2_Wx.txt");//失敗対策確認用
ofstream Test0_MT_C2_Wy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C2_Wy.txt");//失敗対策確認用
ofstream Test0_MT_C1_Wx("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C1_Wx.txt");//失敗対策確認用
ofstream Test0_MT_C1_Wy("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test0_MT_C1_Wy.txt");//失敗対策確認用

ofstream Test_MT_kaisu0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_kaisu0.txt");//失敗対策確認用(取得時間)
ofstream Test_MT_C2_Cx0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Cx0.txt");//失敗対策確認用
ofstream Test_MT_C2_Cy0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Cy0.txt");//失敗対策確認用
ofstream Test_MT_C1_Cx0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Cx0.txt");//失敗対策確認用
ofstream Test_MT_C1_Cy0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Cy0.txt");//失敗対策確認用
ofstream Test_MT_C2_Wx0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Wx0.txt");//失敗対策確認用
ofstream Test_MT_C2_Wy0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C2_Wy0.txt");//失敗対策確認用
ofstream Test_MT_C1_Wx0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Wx0.txt");//失敗対策確認用
ofstream Test_MT_C1_Wy0("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Test_MT_C1_Wy0.txt");//失敗対策確認用

ofstream temp_time("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/temple_time1.txt");//取得時間
ofstream temp_kosuu("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/temp_kosuu.txt");//特徴線の個数

ofstream Goal_RobotX_txt("/home/ros/catkin_ws/src/Struct_SLAM/src/robot/date/Goal_RobotX.txt");//subgoalのx座標
//---------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------Low pass filterここから-----------------------------------------------------------
void Low_pass_filter(float depth_data,int id)
{
  return;
  if (depth_LPF_pre[id] != 0)
  {
    depth[id] = (LPF_T-1)*(depth_LPF_pre[id])/LPF_T + depth_raw_pre[id]/LPF_T; 
  }
  
  depth_LPF_pre[id] = depth[id];
  depth_raw_pre[0] = depth_data;
  
}

//-----------------------------------------------------Low pass filterここまで------------------------------------------------------------



//----------------------------------------------------subgoal決定ここから----------------------------------------------------------------
// void subgoal(cv::Mat img_depth){
//   int max_img_depth = img_depth.at<float>(239,0);
//   int max_pixel_x = 0;
//   float maximum_depth_x;
//   float maximum_depth_y;
//   int i;
//   for(i = 1;i<640;i++){
//     if(max_img_depth<img_depth.at<float>(239,i)){
//       max_img_depth = img_depth.at<float>(239,i);
//       max_pixel_x = i;
//     }
//   }
//   maximum_depth_x = -max_img_depth*(2*max_pixel_x-640)/(2*615.337);
//   maximum_depth_y = -max_img_depth*(2*239-480)/(2*615.458);
//   subgoalY = maximum_depth_y + Est_RobotY;
//   subgoalX = maximum_depth_x + Est_RobotX;
//   decide_subgoal = true;
//   printf("max_img_depth = %f\n",max_img_depth);
//   printf("一番大きいpixel = %f\n",i);
// }
//----------------------------------------------------subgoal決定ここまで---------------------------------------------------------------------


//コールバック関数
void callback(const nav_msgs::Odometry::ConstPtr& msg,const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(time0 != false){
    time_t diffsec = difftime(startTime.tv_sec,endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    ALLrealsec=ALLrealsec+realsec;
    printf("処理の時間=%f\n", realsec);
    printf("処理時間合計=%f\n", ALLrealsec);
  }

	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat depthimage;//Depth画像
	cv::Mat image;//RGB画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
  //cv_bridge処理-----------------------------------------------------------------------------------------------
  //トピックデータのままでは画像処理ができないのでcv_brigeを用いてcv::Mat形式に変換を行う
  try{//MAT形式変換
     bridgeImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
     ROS_INFO("callBack");//printと秒数表示
  }
	//エラー処理
  catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
      //std::cout<<"depth_image_callback Error \n";
      ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
      return ;
  }
  try{//MAT形式変換
    bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
    ROS_INFO("callBack");
  }
	//エラー処理
  catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
    //std::cout<<"depth_image_callback Error \n";
    ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
  return ;}

  cv::Mat img_src = bridgeImage->image.clone();//image変数に変換した画像データを代入
  cv::Mat img_depth = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
  //-----------------------------------------------------------------------------------------------




  cv::Mat img_gray,img_dst,img_fld,img_line2,img_line3,img_line4;//初期定義

  //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
  img_fld = img_src.clone();
  img_fld = cv::Scalar(255,255,255);
  img_line2 = img_src.clone();
  img_line2 = cv::Scalar(255,255,255);
  img_line3 = img_src.clone();
  img_line3 = cv::Scalar(255,255,255);
  img_line4 = img_src.clone();
  img_line4 = cv::Scalar(255,255,255);
  img_FLD_TY = img_src.clone();
  img_FLD_TY = cv::Scalar(255,255,255);
  img_FLD_T = img_src.clone();
  img_FLD_T = cv::Scalar(255,255,255);
  img_FLD_Y = img_src.clone();
  img_FLD_Y = cv::Scalar(255,255,255);
  img_graph= img_src.clone();
  img_graph= cv::Scalar(255,255,255);
  img_tate = img_src.clone();

  //線の描写--------------------------------------------------------------------------------------
  //角度のグラフを作成(ここは消してもいい)
  //グラフは横軸がdegのグラフ
  cv::line(img_graph,cv::Point(0,480),cv::Point(640,480),cv::Scalar(0,0,0), 3, cv::LINE_AA);//X軸 
  cv::line(img_graph,cv::Point(0,480),cv::Point(0,0),cv::Scalar(0,0,0), 3, cv::LINE_AA);//Y軸 
  //cv::line(img_graph,cv::Point(180*3,0),cv::Point(180*3,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
  //cv::line(img_graph,cv::Point(170*3,0),cv::Point(170*3,480),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)
  //cv::line(img_graph,cv::Point(100*3,0),cv::Point(100*3,480),cv::Scalar(0,255,0), 1, cv::LINE_AA);//180度(180*3)
  cv::line(img_graph,cv::Point(90*6,0),cv::Point(90*6,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)add_executable(FLD_marker2_2_6_1 src/FLD/FLD_marker2_2_6_1.cpp)
    
  img_src.copyTo(img_dst);
  img_src.copyTo(img_dst2);
  cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);//グレー画像の作成


  //マーカー検出------------------------------------------------------------------------------------------  
  //ここではArucoマーカーを用いてマーカーの検出を行う
  //マーカーの全体数ALLMarkerは事前に決める
  //検出されたマーカー個数はmarkerIds.size()

  //カメラ内部パラメータ読み込み
  cv::Mat cameraMatrix;
  cv::FileStorage fs;
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
  fs.open("/home/ros/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fs["intrinsic"]>>cameraMatrix;
  intrinsic_K=cameraMatrix;

  //カメラの歪みパラメータ読み込み
  cv::Mat distCoeffs;
  cv::FileStorage fd;
  fd.open("/home/ros/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fd["distortion"]>>distCoeffs;

  //マーカ辞書作成 6x6マスのマーカを250種類生成
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  //charucoボード生成 10x7マスのチェスボード、グリッドのサイズ0.04f、グリッド内マーカのサイズ0.02f
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 7, 0.04f, 0.02f, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();//マーカー検出時メソッドを指定
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

  //マーカー検出
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f> > markerCorners;
  cv::aruco::detectMarkers(img_src, board->dictionary, markerCorners, markerIds, params);

  std::vector<cv::Vec3d> rvecs,tvecs;//マーカーの姿勢(回転ベクトル、並進ベクトル)
  cv::Mat_<double> rvecs2[50],jacobian[50];
  int MarkerC[50][4][2];//マーカーのコーナー座標を記録する([マーカーID][コーナー番号][XorY])
  float MCx[50],MCy[50];//マーカーの中心座標
  float depth0,depthFT;//Depth修正用

  //MCpoint[i][3],MC_point_prve[][][3]はデータ有無の要素（[][][3]=1ならデータ有り,0ならデータ無し)
  if(kaisu==0){//初回のみ全部初期化
    for(int i=0;i<ALLMarker;i++){
      for(int j=0;j<4;j++){
        MC_point[i][3]=0;//全マーカーのデータ確認用要素初期化
        MC_point_prve[i][3]=0;
        //xEst_prev_clP[i]=0;
      }
    }
  }
  //毎回最新pointのみ初期化
  for(int i=0;i<ALLMarker;i++){
    for(int j=0;j<4;j++){
      MC_point[i][3]=0;//全マーカーのデータ確認用要素初期化
    }
  }

  //マーカー観測可能な場合
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(img_dst, markerCorners, markerIds);//マーカー位置を描画
    cv::aruco::drawDetectedMarkers(img_tate, markerCorners, markerIds);//マーカー位置を描画
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);//マーカーの姿勢推定
    for(int i=0;i<markerIds.size();i++){
      //std::cout <<"マーカーの個数:markerIds.size()="<<markerIds.size() << std::endl;//マーカー個数
      //std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;
      MCx[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][0]+MarkerC[markerIds.at(i)][1][0])/2;//マーカー中心座標(x座標)
      MCy[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][1]+MarkerC[markerIds.at(i)][2][1])/2;//マーカー中心座標(y座標)
	    cv::circle(img_dst, cv::Point(MCx[markerIds.at(i)],MCy[markerIds.at(i)]), 3, Scalar(0,255,0),  -1, cv::LINE_AA);//緑点
	    cv::circle(img_tate, cv::Point(MCx[markerIds.at(i)],MCy[markerIds.at(i)]), 3, Scalar(0,255,0),  -1, cv::LINE_AA);//緑点
      cv::aruco::drawAxis(img_dst,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      cv::aruco::drawAxis(img_tate,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

      //画像→カメラ座標変換(マーカーの中心座標を使用)----------------------------------------------------------------------
      pixel[markerIds.at(i)][0]=MCx[markerIds.at(i)];
      pixel[markerIds.at(i)][1]=MCy[markerIds.at(i)];
      //std::cout <<"MCx["<<markerIds.at(i)<<"]="<<MCx[markerIds.at(i)]<<",MCy["<<markerIds.at(i)<<"]="<<MCy[markerIds.at(i)]<< std::endl;
      //std::cout <<"pixel[markerIds.at(i)][0]="<<pixel[markerIds.at(i)][0]<<", pixel[markerIds.at(i)][1]="<<pixel[markerIds.at(i)][1]<< std::endl;//マーカー個数

      if(pixel[markerIds.at(i)][0]>=0&&pixel[markerIds.at(i)][1]>=0){
        depth[markerIds.at(i)] = img_depth.at<float>(cv::Point(pixel[markerIds.at(i)][0],pixel[markerIds.at(i)][1]));

        Low_pass_filter(depth[markerIds.at(i)],markerIds.at(i));

        //Depth値を修正
        //depth0=depth[markerIds.at(i)]*0.001;
        //depthFT=39.215*depth0*depth0-27.793*depth0-7.7718;
        //depth[markerIds.at(i)]=depth[markerIds.at(i)]-depthFT;
        //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
        std::cout<<"depth================================="<<depth[markerIds.at(i)]<<std::endl;
        //std::cout<<"depth1================================="<<depth[1]<<std::endl;
        //std::cout<<"depth2================================="<<depth[2]<<std::endl;
        //std::cout<<"depth3================================="<<depth[3]<<std::endl;
        //std::cout<<"depth4================================="<<depth[4]<<std::endl;
        //std::cout<<"depth5================================="<<depth[5]<<std::endl;
        //std::cout<<"depth6================================="<<depth[6]<<std::endl;

        if(depth[markerIds.at(i)]>0&&depth[markerIds.at(i)]<10000){
            x = (pixel[markerIds.at(i)][0] - 324.473) / 615.337;//ここで正規化座標もしてる
            y = (pixel[markerIds.at(i)][1] - 241.696) / 615.458;
            //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ
            //ロボット座標系(tf座標系)とカメラ観測座標系ではxとyの符号が逆なので注意(ロボット(tf)座標:Y = カメラ観測座標:-X,ロボット(tf)座標:Z = カメラ観測座標:-Y )
            //std::cout <<"x="<< x<< std::endl;
            //std::cout <<"y="<< y<< std::endl;

            MC_point[markerIds.at(i)][0] = -depth[markerIds.at(i)] * x/1000;//メートル表示変換
            MC_point[markerIds.at(i)][1] = -depth[markerIds.at(i)] * y/1000;
            MC_point[markerIds.at(i)][2] = depth[markerIds.at(i)]/1000;
            MC_point[markerIds.at(i)][3] = 1;//データ取得可能なら1
            //std::cout << "マーカーのカメラ座標:MC_point["<<markerIds.at(i)<<"]={"<< MC_point[markerIds.at(i)][0] <<","<<MC_point[markerIds.at(i)][1]<<","<<MC_point[markerIds.at(i)][2]<<"}"<< std::endl;
        }
        //マーカーまでの距離と角度を求める(観測値)
        CameraLM[markerIds.at(i)][0]=sqrt((MC_point[markerIds.at(i)][0]*MC_point[markerIds.at(i)][0])+(MC_point[markerIds.at(i)][2]*MC_point[markerIds.at(i)][2]));
        CameraLM[markerIds.at(i)][1]=atan2(MC_point[markerIds.at(i)][0],MC_point[markerIds.at(i)][2]);
        std::cout <<"CameraLM["<<markerIds.at(i)<<"][0]="<<CameraLM[markerIds.at(i)][0]<< std::endl;//マーカーまでの距離
        std::cout <<"CameraLM["<<markerIds.at(i)<<"][1]="<<CameraLM[markerIds.at(i)][1]<< std::endl;//マーカーまでの角度
        //camera_robot<< "ALLrealsec=" <<ALLrealsec<< " ,realsec=" <<realsec<<" ,CameraLM["<<markerIds.at(i)<<"][0]=" <<CameraLM[markerIds.at(i)][0]<<",CameraLM["<<markerIds.at(i)<<"][1]=" <<CameraLM[markerIds.at(i)][1]<<"\n";

        //カルマンフィルタの観測残差確認用
        Zu[markerIds.at(i)] = (cv::Mat_<double>(2,1) <<
          CameraLM[markerIds.at(i)][0],
          CameraLM[markerIds.at(i)][1]);
      }
    }
  }
  else{std::cout <<"マーカー観測不能"<< std::endl;}
  //ここまでがマーカー観測----------------------------------------------

  //ここから線検出--------------------------------------------------------------------------------------------------------------
  //線検出にはFLDを使用
  //線検出処理後、斜めの線のみを除去してから縦線と横線を求める

    //FLD----------------------------------------------------------------------------
    std::vector<cv::Vec4f> lines_fld;//初期定義
    std::vector<cv::Vec4f> lines_std;//初期定義
  
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines_fld);//特徴線検索
    fld->detect( img_gray, lines_std);//特徴線検索
    
    //FLDの線描写
    for(int i = 0; i < lines_fld.size(); i++){
       //cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_fld,cv::Point(lines_fld[i][0],lines_fld[i][1]),cv::Point(lines_fld[i][2],lines_fld[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_line2,cv::Point(lines_fld[i][0],lines_fld[i][1]),cv::Point(lines_fld[i][2],lines_fld[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }
    //std::cout <<"lines_fld.size()="<<lines_fld.size()<< std::endl;//線の取得数
    //--------------------------------------------------------------------------------

    //線の角度を求める---------------------------------------------------------------------
    double lines2[lines_fld.size()][4],theta0,theta90;//初期定義
    float dep1[lines_fld.size()],dep2[lines_fld.size()];//初期定義

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    
    
    //FLD抽出線とY軸との角度を求める+三次元距離データの結合
    //std::cout <<"並び替え前"<<std::endl;
    int ksk=0;
    for(int i =  0; i < lines_fld.size(); i++){
        dep1[i]= img_depth.at<float>(cv::Point(lines_fld[i][0],lines_fld[i][1]));//点の三次元距離データ取得//20210908ここでDepth情報を取得する必要はない気がする
        dep2[i]= img_depth.at<float>(cv::Point(lines_fld[i][2],lines_fld[i][3]));
        if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
          lines2[ksk][0]=lines_fld[i][0];//ここで距離０を除いてる
          lines2[ksk][1]=lines_fld[i][1];
          lines2[ksk][2]=lines_fld[i][2];
          lines2[ksk][3]=lines_fld[i][3];
          dep1[ksk]=dep1[i];
          dep2[ksk]=dep2[i];
          ksk=ksk+1;//距離データを正しく持つ線のみを取得
        }
    }
    //std::cout <<"テスト:Depthデータが取得可能な線の個数ksk="<<ksk<<std::endl;
    //------------------------------------------------------------------------------------------

    double lines_NNM[lines_fld.size()][4],lines_NNM2[lines_fld.size()][4],lines_NNM_lc[lines_fld.size()],lines_NNM_thetal[lines_fld.size()];
    double lines3[lines_fld.size()][4],lines3_dep[lines_fld.size()][2],lines3_theta[lines_fld.size()];//抽出されたナナメの線以外
    double NNMA[lines_fld.size()],NNMB[lines_fld.size()],NNM_TATE_X[lines_fld.size()][lines_fld.size()],NNM_TATE_Y[lines_fld.size()][lines_fld.size()];
    int lines_NNM_count=0,lines3_count=0;
	  double minlength = image.cols * image.cols * 0.02 ;// (線の最小長さの要件を定義する)
    double NNM_A[lines_fld.size()],NNM_C[lines_fld.size()],NNM_A_MAX,NNM_C_MAX;//斜め線の一次関数
    double NNM_XN[lines_fld.size()],NNM_YN[lines_fld.size()],NNM_CHK[lines_fld.size()];//縦線の結合用
    double NNM_L[lines_fld.size()],tempx,tempy;//縦線の長さ
    double NNM_line[lines_fld.size()][4];//縦線結合後
    int nnmno=0;//縦線結合後個数

    //ナナメの線の抽出を行う-----------------------------------------------------------------------------------
    for(int i=0; i<ksk; i++){
	    //(ほぼ垂直の場合は無視)
      //ここでは斜めの線と斜めではない線に分類分けを行なっている-------------------------------------------------
      //lines3=斜めではない線,lines_NNM=斜めの線
	    if ( abs(lines2[i][0]-lines2[i][2]) < 10 || abs(lines2[i][1]-lines2[i][3]) < 10){ //check if almost vertical
            lines3[lines3_count][0]=lines2[i][0];//ナナメの線以外を抽出する
            lines3[lines3_count][1]=lines2[i][1];
            lines3[lines3_count][2]=lines2[i][2];
            lines3[lines3_count][3]=lines2[i][3];
            lines3_dep[lines3_count][0]=dep1[ksk];
            lines3_dep[lines3_count][1]=dep2[ksk];
            cv::line(img_FLD_TY,cv::Point(lines3[lines3_count][0],lines3[lines3_count][1]),cv::Point(lines3[lines3_count][2],lines3[lines3_count][3]),cv::Scalar(0,0,0), 2, cv::LINE_AA);

            //FLD抽出線のy軸との角度を求める
            lines3_theta[lines3_count]=M_PI-atan2((lines3[lines3_count][2]-lines3[lines3_count][0]),(lines3[lines3_count][3]-lines3[lines3_count][1]));
            //std::cout <<"FLDの線の傾きlines3_theta["<<lines3_count<<"]("<<lines3_theta[lines3_count]<<")"<< std::endl;
            lines3_count=lines3_count+1;//ナナメの線以外を線を数える
			  continue;
      }
		  //(短い線を無視する (x1-x2)^2 + (y2-y1)^2 < minlength)
		  if( ((lines2[i][0]-lines2[i][2])*(lines2[i][0]-lines2[i][2]) +(lines2[i][1]-lines2[i][3])*(lines2[i][1]-lines2[i][3])) < minlength){
		  	continue;
      }   
      //-------------------------------------------------------------------------------------------------------------
      lines_NNM[lines_NNM_count][0]=lines2[i][0];//lines_NNM抽出した斜めの線
      lines_NNM[lines_NNM_count][1]=lines2[i][1];
      lines_NNM[lines_NNM_count][2]=lines2[i][2];
      lines_NNM[lines_NNM_count][3]=lines2[i][3];

       cv::line(img_line2,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 
       //cv::line(img_line4,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 
       cv::line(img_dst,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA); 

      //座標から一次関数を引く関数
      lines_NNM_thetal[lines_NNM_count]=(M_PI/2)-(M_PI-atan2((lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0]),(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1])));
      lines_NNM_lc[lines_NNM_count]=(lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0])*(lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0])+(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1])*(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1]);
      lines_NNM2[lines_NNM_count][0]=lines_NNM[lines_NNM_count][0]+(cos(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*1000;//X1座標
      lines_NNM2[lines_NNM_count][1]=lines_NNM[lines_NNM_count][1]+(sin(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*1000;//Y1座標
      lines_NNM2[lines_NNM_count][2]=lines_NNM[lines_NNM_count][0]+(cos(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*-1000;//X2座標
      lines_NNM2[lines_NNM_count][3]=lines_NNM[lines_NNM_count][1]+(sin(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*-1000;//Y2座標

      cv::line(img_line2,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
      cv::line(img_line4,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
      cv::line(img_FLD_TY,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
      cv::line(img_dst,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
      NNM_CHK[lines_NNM_count]=0;//斜め線判別要素初期化（斜め線の結合に使用)
      lines_NNM_count=lines_NNM_count+1;//ナナメの線の数をカウント
	}//------------------------------------------------------------------------------------------------------------------------(ナナメ線抽出)

  //以下の処理は今回のシステムでは使用していないがもし今後斜めの線を使用する場合は下記の動作が役に立つと思う
  //斜め線のグループ化-----------------------------------------------------------------------------------------------------------
    for (int i=0; i<lines_NNM_count; ++i) {
        //lines_NNM[i][1]の位置を上にする
        if(lines_NNM[i][1]>lines_NNM[i][3]){
            tempx=lines_NNM[i][0];
            tempy=lines_NNM[i][1];
            lines_NNM[i][0]=lines_NNM[i][2];
            lines_NNM[i][1]=lines_NNM[i][3];
            lines_NNM[i][2]=tempx;
            lines_NNM[i][3]=tempy;
        }
        NNM_L[i]=sqrt((lines_NNM[i][0]-lines_NNM[i][2])*(lines_NNM[i][0]-lines_NNM[i][2])
         +(lines_NNM[i][1]-lines_NNM[i][3])*(lines_NNM[i][1]-lines_NNM[i][3]));//縦線の点間の長さ
        NNM_XN[i]=(lines_NNM[i][0]+lines_NNM[i][2])/2;//縦線の中点N
        NNM_YN[i]=(lines_NNM[i][1]+lines_NNM[i][3])/2;
        NNM_A[i]=(lines_NNM[i][1]-lines_NNM[i][3])/(lines_NNM[i][0]-lines_NNM[i][2]);
        NNM_C[i]=lines_NNM[i][1]-(((lines_NNM[i][1]-lines_NNM[i][3])/(lines_NNM[i][0]-lines_NNM[i][2]))*lines_NNM[i][0]);
    }

    double clusNNM_R=0.5;//しきい値
     //角度が類似する線をまとめる
    for (int j=0; j<lines_NNM_count; ++j) {
        if(NNM_CHK[j]==0){
            //std::cout <<"斜めの線["<<j<<"]"<< std::endl;
            int equal=1;//同じ線の要素数
            double NNM_MAX_LONG=NNM_L[j];
            double NNM_MIN=lines_NNM[j][1];
            double NNM_MAX=lines_NNM[j][3];
            NNM_line[nnmno][0]=lines_NNM[j][0],NNM_line[nnmno][1]=lines_NNM[j][1];//基準はjの線(更新が起きなければj単体)
            NNM_line[nnmno][2]=lines_NNM[j][2],NNM_line[nnmno][3]=lines_NNM[j][3];

            for (int k=j+1; k<lines_NNM_count; ++k) {
                //角度の差がしきい値以下の時同じ線とみなす
                if(abs(lines_NNM_thetal[j]-lines_NNM_thetal[k])<clusNNM_R){
                    //std::cout <<"同じ線とみなすlines_NNM["<<k<<"]="<<lines_NNM_thetal[j]-lines_NNM_thetal[k]<< std::endl;
                    NNM_CHK[k]=1;//同じ線とみなす
                    //Y最小が直線の下の端点、Y最大が上の端点になる
                    if(NNM_MIN>lines_NNM[k][1]){
                        NNM_MIN=lines_NNM[k][1];
                        NNM_line[nnmno][1]=lines_NNM[k][1];//Y最小が直線の上の端点
                    }
                    if(NNM_MAX<lines_NNM[k][3]){
                        NNM_MAX=lines_NNM[k][3];
                        NNM_line[nnmno][3]=lines_NNM[k][3];//Y最大が下の端点
                    }
                    //長さが最も長い直線を見つける(長い方がデータに信頼性がある)
                    if(NNM_MAX_LONG<NNM_L[k]){
                        NNM_MAX_LONG=NNM_L[k];//最大長さ更新
                        NNM_A_MAX=NNM_A[k];//最大長さの一次関数を保存
                        NNM_C_MAX=NNM_C[k];//最大長さの一次関数を保存
                    }
                }
            }
            //最も長い線の一次関数からy最大、最小のxを求める
            NNM_line[nnmno][0]=(NNM_line[nnmno][1]-NNM_C_MAX)/NNM_A_MAX;
            NNM_line[nnmno][2]=(NNM_line[nnmno][3]-NNM_C_MAX)/NNM_A_MAX;
            //cv::line(img_dst,cv::Point(NNM_line[nnmno][0],NNM_line[nnmno][1]),cv::Point(NNM_line[nnmno][2],NNM_line[nnmno][3]),cv::Scalar(0,255,255), 4, cv::LINE_AA);
            nnmno=nnmno+1;//縦線の合計個数(まとめ後)
        }
    }
    //std::cout <<"縦線のクラスタ数(まとめ前)lines_NNM_count="<<lines_NNM_count<< std::endl;
    //std::cout <<"縦線の合計個数(まとめ後)tateno="<<nnmno<< std::endl;
    //-------------------------------------------------------------------------------------------------------------------------------斜めの線終了



    //縦線と横線の処理---------------------------------------------------------------------------------------------------------
    //thetaの数値を小さいにソート
    double tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    int yokot,tatet,p,yokoyouso,tateyouso;
    double lines3X,lines3Y,yokolines3[lines_fld.size()][5],tatelines3[lines_fld.size()][5],yokotheta[lines_fld.size()],tatetheta[lines_fld.size()];
    double yokoZ1[lines_fld.size()],yokoZ2[lines_fld.size()],tateZ1[lines_fld.size()],tateZ2[lines_fld.size()];
    double yokothetal[lines_fld.size()],tatethetal[lines_fld.size()],yokolc[lines_fld.size()],tatelc[lines_fld.size()],yokol[lines_fld.size()][4],tatel[lines_fld.size()][4];
    double datat[lines_fld.size()],datay[lines_fld.size()];//ヒストグラムデータ用
    int clusCY,clusCT,clusy[lines_fld.size()],clust[lines_fld.size()],MAXY=0,MAXT=0;//クラスタリング用
    double yokothetaCL[lines_fld.size()],tatethetaCL[lines_fld.size()],clusR;//クラスタリング用
    double yokoclusy[100][200][5],tateclust[100][200][5];
    double Average_yoko_theta[lines_fld.size()];
    double MINI_theta=100,mini_theta;
    int CLUSTER[lines_fld.size()],clus_no=0,YOKO_CLUST;
    int nanamet=0,nanamey=0,NANAME_Line[100];
    double lines_T_theta[lines_fld.size()];
    double tateA[lines_fld.size()],tateB[lines_fld.size()];
    double TATE_A[lines_fld.size()],TATE_C[lines_fld.size()],TATE_A_MAX,TATE_C_MAX;//縦線の一次関数y=TATE_Ax+TATE_C
    double TATE_D[lines_fld.size()][lines_fld.size()],TATE_XN[lines_fld.size()],TATE_YN[lines_fld.size()];//縦線の結合用
    double TATE_L[lines_fld.size()],TATE_K[lines_fld.size()];//縦線の長さ
    double TATE_line[lines_fld.size()][4];//縦線結合後
    int tateno=0,tateok=0;//縦線結合後個数
    
    yokot=0,tatet=0,p=0,yokoyouso=0,tateyouso=0;

    //縦線と横線の分類分け----------------------------------------------------------------------------------------------------------------
    for (int j=0; j< lines3_count; ++j) {
        lines3X=abs(lines3[j][0]-lines3[j][2]);//傾きを調べる（x成分)
        lines3Y=abs(lines3[j][1]-lines3[j][3]);//傾きを調べる（y成分)
        
        //横線に分類
        if(lines3X>lines3Y){
            //std::cout <<"yoko(lines3X>lines3Y)="<<lines3X<<">"<<lines3Y<< std::endl;
            //std::cout <<"lines3_theta["<<j<<"](ヨコ)="<<lines3_theta[j]<< std::endl;

            yokolines3[yokoyouso][0]=lines3[j][0];//(x成分)
            yokolines3[yokoyouso][1]=lines3[j][1];//(y成分)
            yokolines3[yokoyouso][2]=lines3[j][2];//(x成分)
            yokolines3[yokoyouso][3]=lines3[j][3];//(y成分)
            yokolines3[yokoyouso][4]=lines3_theta[j];

            //yokotheta[yokoyouso]=theta[j];
            yokoZ1[yokoyouso]=lines3_dep[j][0];
            yokoZ2[yokoyouso]=lines3_dep[j][1];
            //cv::line(img_line3,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
            //cv::line(img_FLD_Y,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
            cv::line(img_line2,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
          	//cv::circle(img_dst, cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]), 4, cv::Scalar(255, 0, 255), 1.5);
          	//cv::circle(img_dst, cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]), 4, cv::Scalar(255, 0, 255), 1.5);

            yokotheta[yokoyouso]=lines3_theta[j]*180/M_PI;//deg表示化
            //θの範囲を0〜180にする
            if(yokotheta[yokoyouso]>=180){
                yokotheta[yokoyouso]=yokotheta[yokoyouso]-180;
                yokolines3[yokoyouso][4]=yokolines3[yokoyouso][4]-M_PI;
            }
            //std::cout <<"yokotheta["<<yokoyouso<<"]="<<yokotheta[yokoyouso]<< std::endl;

            if(yokotheta[yokoyouso]>=90){
                yokotheta[yokoyouso]=180-yokotheta[yokoyouso];
            }
            else{yokotheta[yokoyouso]=yokotheta[yokoyouso];}
            //std::cout <<"yokotheta["<<yokoyouso<<"](クラスタリング用)="<<yokotheta[yokoyouso]<< std::endl;

            clusy[yokoyouso]=0;//クラスタリング用
            yokoyouso=yokoyouso+1;//横線に分類されたグループ数(yokoyouso)
        }
        //縦線に分類
        else{
            if(lines3Y>lines3X){
                //std::cout <<"tate(lines3Y>lines3X)="<<lines3Y<<">"<<lines3X<< std::endl;
                //std::cout <<"lines3_theta["<<j<<"](タテ)="<<lines3_theta[j]<< std::endl;
                tatelines3[tateyouso][0]=lines3[j][0];
                tatelines3[tateyouso][1]=lines3[j][1];
                tatelines3[tateyouso][2]=lines3[j][2];
                tatelines3[tateyouso][3]=lines3[j][3];
                tatelines3[tateyouso][4]=lines3_theta[j];

                //tatetheta[tateyouso]=lines3_theta[j];
                tateZ1[tateyouso]=lines3_dep[j][0];
                tateZ2[tateyouso]=lines3_dep[j][1];
                //cv::line(img_line3,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                //cv::line(img_FLD_T,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                cv::line(img_line2,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);

                //確率ハフ変換を使用しない時
                tatetheta[tateyouso]=lines3_theta[j]*180/M_PI;//deg表示化

                //θの範囲を0〜180にする
                if(tatetheta[tateyouso]>=180){
                    tatetheta[tateyouso]=tatetheta[tateyouso]-180;
                    tatelines3[tateyouso][4]=tatelines3[tateyouso][4]-M_PI;
                    }
                //std::cout <<"tatetheta["<<tateyouso<<"]="<<tatetheta[tateyouso]<< std::endl;

                //クラスタリング時に最大個数を持つ縦線のクラスタが２つ存在してしまうため、90度で反転させてクラスタリング処理を行う。
                //例(θ=0~10,170~180→180-170=10)
                if(tatetheta[tateyouso]>=90){
                    tatetheta[tateyouso]=180-tatetheta[tateyouso];
                }
                else{tatetheta[tateyouso]=tatetheta[tateyouso];}
                //std::cout <<"tatetheta["<<tateyouso<<"](クラスタリング用)="<<tatetheta[tateyouso]<< std::endl;

                clust[tateyouso]=0;//クラスタリング用
                tateyouso=tateyouso+1;//縦線に分類されたグループ数(tateyouso)
            }
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------------------

    //ここの並び替え上の並び替えとまとめられそう（先に範囲を狭めてから並び替えして分類する感じにしたらできそう）
    //今は並び替えして分類して範囲狭めて再び並び替えしてる
    //クラスタリング用θで並び替えを行う
    //tatethetaの数値を小さいにソート
    tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    for (int i=0; i<tateyouso; ++i) {
       for (int j=i+1;j<tateyouso; ++j) {
            if (tatetheta[i] > tatetheta[j]) {
                tmp =  tatetheta[i];
                tmp1x =  tatelines3[i][0];
                tmp1y =  tatelines3[i][1];
                tmp2x =  tatelines3[i][2];
                tmp2y =  tatelines3[i][3];
                tmp3 =  tatelines3[i][4];

                tatetheta[i] = tatetheta[j];
                tatelines3[i][0] = tatelines3[j][0];
                tatelines3[i][1] = tatelines3[j][1];
                tatelines3[i][2] = tatelines3[j][2];
                tatelines3[i][3] = tatelines3[j][3];
                tatelines3[i][4] = tatelines3[j][4];

                tatetheta[j] = tmp;
                tatelines3[j][0] = tmp1x;
                tatelines3[j][1] = tmp1y;
                tatelines3[j][2] = tmp2x;
                tatelines3[j][3] = tmp2y;
                tatelines3[j][4] = tmp3;
            }
        }
    }
    
    //クラスタリング用θで並び替えを行う
    //yokothetaの数値を小さいにソート
    tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    for (int i=0; i<yokoyouso; ++i) {
       for (int j=i+1;j<yokoyouso; ++j) {
            if (yokotheta[i] > yokotheta[j]) {
                tmp =  yokotheta[i];
                tmp1x =  yokolines3[i][0];
                tmp1y =  yokolines3[i][1];
                tmp2x =  yokolines3[i][2];
                tmp2y =  yokolines3[i][3];
                tmp3 =  yokolines3[i][4];

                yokotheta[i] = yokotheta[j];
                yokolines3[i][0] = yokolines3[j][0];
                yokolines3[i][1] = yokolines3[j][1];
                yokolines3[i][2] = yokolines3[j][2];
                yokolines3[i][3] = yokolines3[j][3];
                yokolines3[i][4] = yokolines3[j][4];

                yokotheta[j] = tmp;
                yokolines3[j][0] = tmp1x;
                yokolines3[j][1] = tmp1y;
                yokolines3[j][2] = tmp2x;
                yokolines3[j][3] = tmp2y;
                yokolines3[j][4] = tmp3;
            }
        }
    }

    //縦線のクラスタリング-----------------------------------------------------------------------------------------------------------------------
    //タテ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCT:クラスタ数,Clust[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    //clusR=0.15,clusCT=0,Average_tate_theta=0;
    clusR=0.5,clusCT=0,Average_tate_theta=0;
    //要素数が0の時は実行しない
    if(tateyouso==0){
        //std::cout <<"実行しない--------------tateyouso="<<tateyouso<< std::endl;
        tate_histgram = false;//タテ線のヒストグラムを実行しない
    }
    else{
        tate_histgram = true;//タテ線のヒストグラムを実行
        for (int j=0; j< tateyouso; ++j) {
            if(tateyouso==1){//要素がひとつしかないとき比較ができない
                //std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                //std::cout <<"クラスタ番号clusCT="<<clusCT<< std::endl;//クラスタ数
                //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(tatetheta[j]*6, 240), 3, Scalar(0,5*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                tateclust[0][clust[0]][0]=tatelines3[j][0];//(x成分)
                tateclust[0][clust[0]][1]=tatelines3[j][1];//(y成分)
                tateclust[0][clust[0]][2]=tatelines3[j][2];//(x成分)
                tateclust[0][clust[0]][3]=tatelines3[j][3];//(y成分)
                tateclust[0][clust[0]][4]=tatelines3[j][4];//角度
                clust[MAXT]=1;
            }

            else{
                cv::circle(img_graph, cv::Point(tatetheta[j]*6, 240), 3, Scalar(0,5*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        //std::cout <<"初回動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        //cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        //cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新
                    }
                    //クラスタリング範囲外
                    else{
                        //std::cout <<"初回動作範囲外j="<<j<<",clusCT="<<clusCT<<"----------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        //cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        //cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        MAXT=clusCT;
                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //中間動作
                if(j!=0&&j+1!=tateyouso){
                    //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //後方クラスタリング半径範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        //std::cout <<"中間動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        //cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        //cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新
                    }
                    //後方クラスタリング半径範囲外
                    else{
                        //std::cout <<"中間動作範囲外j="<<j<<",clusCT="<<clusCT<<"--------------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        //cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        //cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //最終動作
                if(j+1==tateyouso){
                    //std::cout <<"最終動作j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                    tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                    tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                    tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                    tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                    //cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                    //cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                    //std::cout <<"最終:最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<< std::endl;
                }
                if(clust[clusCT]>=clust[MAXT]){ MAXT=clusCT; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<<"\n"<< std::endl;
            }
        }
        //線のグループ化-----------------------------------------------------------------------------------------------------------
        //短い線はノイズなので除去する
        for (int i=0; i<clust[MAXT]; ++i) {
            if(sqrt((tateclust[MAXT][i][0]-tateclust[MAXT][i][2])*(tateclust[MAXT][i][0]-tateclust[MAXT][i][2])
             +(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])*(tateclust[MAXT][i][1]-tateclust[MAXT][i][3]))>40){
                tateclust[MAXT][tateok][0]=tateclust[MAXT][i][0];
                tateclust[MAXT][tateok][1]=tateclust[MAXT][i][1];
                tateclust[MAXT][tateok][2]=tateclust[MAXT][i][2];
                tateclust[MAXT][tateok][3]=tateclust[MAXT][i][3];
                tateok=tateok+1;//縦線の合計個数(まとめ後)
            }
        }
        //Xの値が小さい順に並び変える
        tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
        for (int i=0; i<tateok; ++i) {
           for (int j=i+1;j<tateok; ++j) {
                if (tateclust[MAXT][i][0]+tateclust[MAXT][i][2] > tateclust[MAXT][j][0]+tateclust[MAXT][j][2]) {
                    tmp1x =  tateclust[MAXT][i][0];
                    tmp1y =  tateclust[MAXT][i][1];
                    tmp2x =  tateclust[MAXT][i][2];
                    tmp2y =  tateclust[MAXT][i][3];
                    tmp3 =  tateclust[MAXT][i][4];

                    tateclust[MAXT][i][0] = tateclust[MAXT][j][0];
                    tateclust[MAXT][i][1] = tateclust[MAXT][j][1];
                    tateclust[MAXT][i][2] = tateclust[MAXT][j][2];
                    tateclust[MAXT][i][3] = tateclust[MAXT][j][3];
                    tateclust[MAXT][i][4] = tateclust[MAXT][j][4];

                    tateclust[MAXT][j][0] = tmp1x;
                    tateclust[MAXT][j][1] = tmp1y;
                    tateclust[MAXT][j][2] = tmp2x;
                    tateclust[MAXT][j][3] = tmp2y;
                    tateclust[MAXT][j][4] = tmp3;
                }
            }
            //tateclust[MAXT][j][1]の位置を上にする
            if(tateclust[MAXT][i][1]>tateclust[MAXT][i][3]){
                tempy=tateclust[MAXT][i][1];
                tempx=tateclust[MAXT][i][0];
                tateclust[MAXT][i][1]=tateclust[MAXT][i][3];
                tateclust[MAXT][i][0]=tateclust[MAXT][i][2];
                tateclust[MAXT][i][3]=tempy;
                tateclust[MAXT][i][2]=tempx;
            }
            TATE_L[i]=sqrt((tateclust[MAXT][i][0]-tateclust[MAXT][i][2])*(tateclust[MAXT][i][0]-tateclust[MAXT][i][2])
             +(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])*(tateclust[MAXT][i][1]-tateclust[MAXT][i][3]));//縦線の点間の長さ
            TATE_XN[i]=(tateclust[MAXT][i][0]+tateclust[MAXT][i][2])/2;//縦線の中点N
            TATE_YN[i]=(tateclust[MAXT][i][1]+tateclust[MAXT][i][3])/2;
        }

        double clusTATE_R=10;//しきい値
        //縦線の中点と直線の距離を使用し、直線をまとめる(距離がしきい値以内なら同じ線とみなす)
       for (int j=0; j<tateok; ++j) {
            int equal=0;//同じ線の要素数
            TATE_line[tateno][0]=tateclust[MAXT][j][0],TATE_line[tateno][1]=tateclust[MAXT][j][1];//基準はjの線(更新が起きなければj単体)
            TATE_line[tateno][2]=tateclust[MAXT][j][2],TATE_line[tateno][3]=tateclust[MAXT][j][3];
            double TATE_MAX_LONG=TATE_L[j];
            double TATE_MIN=tateclust[MAXT][j][1];
            double TATE_MAX=tateclust[MAXT][j][3];

            //縦線がy軸と平行でない時のみ実行
            if(tateclust[MAXT][j][0]!=tateclust[MAXT][j][2]){
                //std::cout <<"縦線が斜めのとき:tatethetal["<<j<<"]"<< std::endl;

                TATE_A[j]=(tateclust[MAXT][j][1]-tateclust[MAXT][j][3])/(tateclust[MAXT][j][0]-tateclust[MAXT][j][2]);
                TATE_C[j]=tateclust[MAXT][j][1]-(((tateclust[MAXT][j][1]-tateclust[MAXT][j][3])/(tateclust[MAXT][j][0]-tateclust[MAXT][j][2]))*tateclust[MAXT][j][0]);
                for (int k=j+1; k<tateok; ++k) {
                    TATE_D[j][k]=abs((TATE_A[j]*TATE_XN[k])-TATE_YN[k]+TATE_C[j])/sqrt((TATE_A[j]*TATE_A[j])+1);//点と直線の距離
                    //距離がしきい値以下の時同じ線とみなす
                    if(TATE_D[j][k]<clusTATE_R){
                        //std::cout <<"(斜め)同じ線とみなすTATE_D["<<j<<"]["<<k<<"]="<<TATE_D[j][k]<< std::endl;
                        //Y最小が直線の下の端点、Y最大が上の端点になる(y軸並行なのでxは長い方の値を使用)
                        if(TATE_MIN>tateclust[MAXT][k][1]){
                            TATE_MIN=tateclust[MAXT][k][1];
                            TATE_line[tateno][1]=tateclust[MAXT][k][1];//Y最小が直線の上の端点
                        }
                        if(TATE_MAX<tateclust[MAXT][k][3]){
                            TATE_MAX=tateclust[MAXT][k][3];
                            TATE_line[tateno][3]=tateclust[MAXT][k][3];//Y最大が下の端点
                        }
                        //長さが最も長い直線にまとめる(長い方がデータに信頼性がある)
                        if(TATE_MAX_LONG<TATE_L[k]){
                            TATE_MAX_LONG=TATE_L[k];//最大長さ更新
                            TATE_A_MAX=(tateclust[MAXT][k][1]-tateclust[MAXT][k][3])/(tateclust[MAXT][k][0]-tateclust[MAXT][k][2]);
                            TATE_C_MAX=tateclust[MAXT][k][1]-(((tateclust[MAXT][k][1]-tateclust[MAXT][k][3])/(tateclust[MAXT][k][0]-tateclust[MAXT][k][2]))*tateclust[MAXT][k][0]);
                            //最も長い線の一次関数からy最大、最小のxを求める
                            TATE_line[tateno][0]=(TATE_line[tateno][1]-TATE_C_MAX)/TATE_A_MAX;
                            TATE_line[tateno][2]=(TATE_line[tateno][3]-TATE_C_MAX)/TATE_A_MAX;
                        }
                        equal=equal+1;//同じ線の要素数
                    }
                    else{continue;}//連番で見ているのでしきい値を超えた時点で同じ線は無い
                }
            }
            //縦線がy軸と平行なとき
            else{
                //std::cout <<"縦線がY軸と平行:tatethetal["<<j<<"]"<< std::endl;
                for (int k=j+1; k<tateok; ++k) {
                    TATE_D[j][k]=abs(tateclust[MAXT][j][0]-TATE_XN[k]);//点と直線の距離(y軸並行)
                    //距離がしきい値以下の時同じ線とみなす
                    if(TATE_D[j][k]<clusTATE_R){
                        //std::cout <<"(平行)同じ線とみなすTATE_D["<<j<<"]["<<k<<"]="<<TATE_D[j][k]<< std::endl;
                        //長さが最も長い直線にまとめる(長い方がデータに信頼性がある)
                        if(TATE_MAX_LONG<TATE_L[k]){
                            TATE_MAX_LONG=TATE_L[k];//最大長さ更新
                            TATE_line[tateno][0]=tateclust[MAXT][k][0];//最大長さのxを保存
                            TATE_line[tateno][2]=tateclust[MAXT][k][2];
                            
                        }
                        //Y最小が直線の下の端点、Y最大が上の端点になる(y軸並行なのでxは長い方の値を使用)
                        if(TATE_MIN>tateclust[MAXT][k][1]){
                            TATE_MIN=tateclust[MAXT][k][1];
                            TATE_line[tateno][1]=tateclust[MAXT][k][1];//Y最小が直線の上の端点
                        }
                        if(TATE_MAX<tateclust[MAXT][k][3]){
                            TATE_MAX=tateclust[MAXT][k][3];
                            TATE_line[tateno][3]=tateclust[MAXT][k][3];//Y最大が下の端点
                        }
                        equal=equal+1;//同じ線の要素数
                    }
                    else{continue;}//連番で見ているのでしきい値を超えた時点で同じ線は無い
                }
            }
            cv::circle(img_dst2, cv::Point(TATE_line[tateno][0], TATE_line[tateno][1]), 3, Scalar(255,0,0), -1, cv::LINE_AA);//線の角度グラフ
            cv::circle(img_dst2, cv::Point(TATE_line[tateno][2], TATE_line[tateno][3]), 3, Scalar(0,255,0), -1, cv::LINE_AA);//線の角度グラフ

            //短い線はノイズなので除去する
            if(sqrt((TATE_line[tateno][0]-TATE_line[tateno][2])*(TATE_line[tateno][0]-TATE_line[tateno][2])
             +(TATE_line[tateno][1]-TATE_line[tateno][3])*(TATE_line[tateno][1]-TATE_line[tateno][3]))>40){
                tateno=tateno+1;//縦線の合計個数(まとめ後)
            }
            j=j+equal;//同じ線とみなされた個数分進む
        }
        //std::cout <<"縦線のクラスタ数(まとめ前)clust[MAXT]="<<clust[MAXT]<< std::endl;
        //std::cout <<"縦線の合計個数(まとめ後)tateno="<<tateno<< std::endl;
        //-------------------------------------------------------------------------------------------------------------------------------
        tate_point_curr.resize(1000);//配列初期設定(縦線の中点座標)

        //一次関数を描写するプログラム
        for (int j=0; j<tateno; ++j) {
            //std::cout <<"j="<< j<< std::endl;
            //std::cout <<"clust["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
            //std::cout <<"tateclust["<<MAXT<<"]["<<j<<"][0]="<<TATE_line[j][0]<< std::endl;//確認用

            //座標から一次関数を引く関数
            tatethetal[j]=(M_PI/2)-(M_PI-atan2((TATE_line[j][2]-TATE_line[j][0]),(TATE_line[j][3]-TATE_line[j][1])));
            tatelc[j]=(TATE_line[j][2]-TATE_line[j][0])*(TATE_line[j][2]-TATE_line[j][0])+(TATE_line[j][3]-TATE_line[j][1])*(TATE_line[j][3]-TATE_line[j][1]);
            tatel[j][0]=TATE_line[j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*100;//X1座標
            tatel[j][1]=TATE_line[j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*100;//Y1座標
            tatel[j][2]=TATE_line[j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*-100;//X2座標
            tatel[j][3]=TATE_line[j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*-100;//Y2座標

            cv::line(img_line4,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst2,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_tate,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst2,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
            cv::line(img_tate,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
            cv::line(img_line4,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);

            datat[j]=TATE_line[clust[j]][4];
            //最大クラスタ内の平均角度を求める
            Average_tate_theta=Average_tate_theta+datat[j];

            //縦線の中点を求める
            tate_point_curr[j].x=(TATE_line[j][0]+TATE_line[j][2])/2;
            tate_point_curr[j].y=(TATE_line[j][1]+TATE_line[j][3])/2;
	        cv::circle(img_dst2, cv::Point(tate_point_curr[j]), 3, Scalar(0,255,255),  -1, cv::LINE_AA);
        }
        tate_point_curr.resize(tateno);//リサイズ(縦線の中点座標)

        //最大クラスタの要素数が１つだけの時を考慮
        if(clust[MAXT]>1){Average_tate_theta=Average_tate_theta/(clust[MAXT]-1);}//最大クラスタの要素が２つ以上なら通常の平均計算
        else{Average_tate_theta=Average_tate_theta;}//最大クラスタの要素数が２つ未満ならその値を平均値とする

        cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta),380+100*cos(Average_tate_theta)),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta<<"\n"<< std::endl;
        //cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta/(clust[MAXT]-1)),380+100*cos(Average_tate_theta/(clust[MAXT]-1))),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta/(clust[MAXT]-1)<<"\n"<< std::endl;

        //縦線の法線ベクトルを求める(未使用)-----------------------------------------
         MatrixXd R(2,2);
         R(0,0)=cos(M_PI/2);
         R(0,1)=-sin(M_PI/2);
         R(1,0)=sin(M_PI/2);
         R(1,1)=cos(M_PI/2);

        MatrixXd n(2,1);
        MatrixXd N(2,1);//Nは2行L列の行列
        N(0,0)=100-100*sin(Average_tate_theta);
        N(1,0)=380+100*cos(Average_tate_theta);
        //N(0,0)=100-100*sin(Average_tate_theta/(clust[MAXT]-1));
        //N(1,0)=380+100*cos(Average_tate_theta/(clust[MAXT]-1));

        MatrixXd P0(2,1);//推定交点
        P0(0,0)=(100+(100-100*sin(Average_tate_theta)))/2;
        P0(1,0)=(380+(380+100*cos(Average_tate_theta)))/2;
        //P0(0,0)=(100+(100-100*sin(Average_tate_theta/(clust[MAXT]-1))))/2;
        //P0(1,0)=(380+(380+100*cos(Average_tate_theta/(clust[MAXT]-1))))/2;

        n=R*(P0-N);//直線を90度回転させたベクトルn
        cv::line(img_graph,cv::Point(P0(0,0),P0(1,0)),cv::Point(P0(0,0)-n(0,0),P0(1,0)-n(1,0)),cv::Scalar(255,0,255), 3, cv::LINE_AA);//法線ベクトル

        //縦線の平均方向の法線ベクトルとy軸との角度を求める
        Average_tate_theta_Y=M_PI-atan2(-n(0,0),-n(1,0));
        //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<<"\n"<< std::endl;

        //次はこの法線ベクトルの角度に最も近いヨコ線のクラスタを求める
        //求める方法としては各クラスタの平均角度を求め、その平均角度と比較して最も比較の値が小さいやつをヨコ線のクラスタとする。
        //ただしその比較値がある値以上だった場合はヨコ線の方向は更新せず前ステップでのヨコ線の方向を使う。

        //縦線が検出されなかった場合もまた同様に縦線の方向は前ステップでの方向を利用する
        
    }

    //std::cout <<"\n"<< std::endl;

    //横線のクラスタリング(未使用)--------------------------------------------------------------------------------------------------------
    //ヨコ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCY:クラスタ数,Clusy[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    clusR=0.15,clusCY=0;
    //要素数が0の時は実行しない
    if(yokoyouso==0){
        //std::cout <<"実行しない--------------yokoyouso="<<yokoyouso<< std::endl;
        yoko_histgram = false;//ヨコ線のヒストグラムを実行しない
    }
    else{
        yoko_histgram = true;//ヨコ線のヒストグラムを実行
        for (int j=0; j< yokoyouso; ++j) {
            if(yokoyouso==1){//要素がひとつしかない時は比較ができない(クラスタリングできない)
                //std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                //std::cout <<"クラスタ番号clusCY="<<clusCY<< std::endl;//クラスタ数
                //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(yokotheta[j]*6, 240), 3, Scalar(255,5*clusCY,0), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                yokoclusy[0][clusy[0]][0]=yokolines3[j][0];//(x成分)
                yokoclusy[0][clusy[0]][1]=yokolines3[j][1];//(y成分)
                yokoclusy[0][clusy[0]][2]=yokolines3[j][2];//(x成分)
                yokoclusy[0][clusy[0]][3]=yokolines3[j][3];//(y成分)
                yokoclusy[0][clusy[0]][4]=yokolines3[j][4];
                clusy[MAXY]=1;
            }

            else{//クラスタリング
                cv::circle(img_graph, cv::Point(yokotheta[j]*6, 240), 3, Scalar(255,5*clusCY,0), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        //std::cout <<"初回動作範囲内j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];// 角度
                        //cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        //cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す(初回動作)
                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //クラスタリング範囲外
                    else{
                        //std::cout <<"初回動作範囲外j="<<j<<",クラスタ番号clusCY="<<clusCY<< ",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度
                        
                        //cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        //cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);
                        MAXY=clusCY;

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求める(初回動作クラスタリング範囲外なのでデータが１つしかない)
                        //std::cout <<"初回動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;

                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での平均値初期化
                    }
                }
                //中間動作
                if(j!=0&&j+1!=yokoyouso){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //後方範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        //std::cout <<"中間動作範囲内j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                        //cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        //cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す
                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //後方範囲外
                    else{
                        //std::cout <<"中間動作範囲外j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                        //cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        //cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す
                        
                        if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                        }
                        else{//クラスタの要素が一つしかない時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均=要素(要素がひとつしかないから=平均)
                        }
                        //std::cout <<"中間動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;
                        //要素数が3個以上のクラスタをキープする(開始要素が0を考慮)
                        if(clusy[clusCY]>=3-1){
                            clus_no=clus_no+1;
                            //std::cout <<"要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                            CLUSTER[clus_no]=clusCY;//クラスタ番号を CLUSTER[clus_no]に保存
                        }
                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での次の平均値初期化
                    }
                }
                //最終動作(j<yokoyouso個)
                if(j+1==yokoyouso){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"最終動作j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                    //cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                    //cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                    Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す

                    if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                    }
                    else{//クラスタの要素が一つしかない時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均を求める
                    }
                    //std::cout <<"最終動作:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<< std::endl;
                    //要素数が3個以上のクラスタをキープする
                    if(clusy[clusCY]>=3-1){
                        clus_no=clus_no+1;
                        //std::cout <<"最終動作:要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                        CLUSTER[clus_no]=clusCY;
                    }
                }

                //if(clusy[clusCY]>=clusy[MAXY]){ MAXY=clusCY; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXY=clusCY="<<MAXY<<",最大クラスタの内部個数clusy[MAXY]="<<clusy[MAXY]<< std::endl;
            }
        }
        //クラスタリングの要素数が３つ以上のクラスタがあるときだけ実行
        if(clus_no>0){
            //std::cout <<"テスト:要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
            for(int h=1;h<=clus_no;h++){
                //std::cout <<"要素数が3個以上のクラスタ  クラスタ番号clusCY=CLUSTER[clus_no="<<h<<"]="<<CLUSTER[h]<<std::endl;
                //std::cout <<"Average_yoko_theta["<<CLUSTER[h]<<"]="<<Average_yoko_theta[CLUSTER[h]]<< std::endl;
                //縦線の法線ベクトルとヨコ線の平均角度の差を見てる
                if(MINI_theta>abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]])){
                    MINI_theta=abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]]);
                    YOKO_CLUST=CLUSTER[h];
                }
            }
            //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<< std::endl;
            //std::cout <<"タテの法線ベクトルと最も並列なヨコ線クラスタ clusCY=YOKO_CLUST="<<YOKO_CLUST<<",Average_yoko_theta_Y="<<Average_yoko_theta[YOKO_CLUST]<< std::endl;
            cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_yoko_theta[YOKO_CLUST]),380+100*cos(Average_yoko_theta[YOKO_CLUST])),cv::Scalar(255,0,0), 3, cv::LINE_AA);

            //std::cout <<"clusy[YOKO_CLUST="<<YOKO_CLUST<<"]="<<clusy[YOKO_CLUST]<< std::endl;

            //一次関数を描写するプログラム
            for(int j = 0; j <clusy[YOKO_CLUST]; j++){
                //std::cout <<"j="<< j<< std::endl;
                //std::cout <<"clusy["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
                //std::cout <<"yokoclusy["<<YOKO_CLUST<<"]["<<j<<"][0]="<<yokoclusy[YOKO_CLUST][j][0]<< std::endl;//確認用

                //座標から一次関数を引く関数(線を延長してる)
                yokothetal[j]=(M_PI/2)-(M_PI-atan2((yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0]),(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])));
                yokolc[j]=(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])*(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])+(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])*(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1]);
                yokol[j][0]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*100;//X1座標
                yokol[j][1]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*100;//Y1座標
                yokol[j][2]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*-100;//X2座標
                yokol[j][3]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*-100;//Y2座標

                cv::line(img_line4,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
                cv::line(img_dst,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
                cv::line(img_dst,cv::Point(yokoclusy[YOKO_CLUST][j][0],yokoclusy[YOKO_CLUST][j][1]),
                cv::Point(yokoclusy[YOKO_CLUST][j][2],yokoclusy[YOKO_CLUST][j][3]),cv::Scalar(255,0,0), 4, cv::LINE_AA);
                cv::line(img_line4,cv::Point(yokoclusy[YOKO_CLUST][j][0],yokoclusy[YOKO_CLUST][j][1]),
                 cv::Point(yokoclusy[YOKO_CLUST][j][2],yokoclusy[YOKO_CLUST][j][3]),cv::Scalar(255,0,0), 4, cv::LINE_AA);

                datay[j]=yokoclusy[YOKO_CLUST][j][4];
            }
        }
    }//-------------------------------------------------------------------------------------------------------------------------------------

    /*//タテ線とナナメ線の交点を検出する(未使用)--------------------------------------------------------------------------------------------------
    //縦線の一次関数を求める
    for (int j=0; j<lines_NNM_count; ++j) {
        NNMA[j]=(lines_NNM[j][1]-lines_NNM[j][3])/(lines_NNM[j][0]-lines_NNM[j][2]);
        NNMB[j]=lines_NNM[j][1]-(((lines_NNM[j][1]-lines_NNM[j][3])/(lines_NNM[j][0]-lines_NNM[j][2]))*lines_NNM[j][0]);
        for (int i=0; i<clust[MAXT]; ++i) {
            if(tateclust[MAXT][i][0]!=tateclust[MAXT][i][2]){
                tateA[i]=(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])/(tateclust[MAXT][i][0]-tateclust[MAXT][i][2]);
                tateB[i]=tateclust[MAXT][i][1]-tateA[i]*tateclust[MAXT][i][0];
                NNM_TATE_X[j][i]=(tateB[i]-NNMB[j])/(NNMA[j]-tateA[i]);
                NNM_TATE_Y[j][i]=NNMA[j]*((tateB[i]-NNMB[j])/(NNMA[j]-tateA[i]))+NNMB[j];
            }
            else{
                NNM_TATE_X[j][i]=tateclust[MAXT][i][0];
                NNM_TATE_Y[j][i]=(NNMA[j]*NNM_TATE_X[j][i])+NNMB[j];
            }
            //std::cout <<"縦線とヨコ線の交点["<<j<<"]["<<i<<"]=("<<NNM_TATE_X[j][i]<<","<<NNM_TATE_Y[j][i]<<")"<< std::endl;
            //cv::circle(img_dst,cv::Point(NNM_TATE_X[j][i],NNM_TATE_Y[j][i]),3,Scalar(0,255,255),-1);
        }
    }*/

    //線検出プログラム終了-----------------------------------------------------------------------------------------------------------------------


    //テンプレートマッチング-----------------------------------------------------------------------------------------------------------
    TPCC.resize(1000);//配列初期設定(tate_point_camera_c)
    DTPC_ok=0;
    //テンプレートマッチングプログラム
    //縦線の中点を使用してテンプレートを作成する(線検出の段階ではDepthの不具合などは考慮していないためここで考慮する+クロップ不具合)
    for (int i=0; i<tate_point_curr.size(); ++i) {
        if(template_size<tate_point_curr[i].y&&tate_point_curr[i].y<480-template_size){
            if(template_size<tate_point_curr[i].x&&tate_point_curr[i].x<640-template_size){
                DTPC[i] = img_depth.at<float>(cv::Point(tate_point_curr[i].x,tate_point_curr[i].y));//depth_tate_point_curr=DTPC
                //Depthが取得できない縦線中点を削除する+Depthの外れ値を除く
                if(DTPC[i]>0.001&&DTPC[i]<10000){//tate_point_camera_c=TPCC
                    TPCC[DTPC_ok].x = -DTPC[i] * ((tate_point_curr[i].x - 324.473) / 615.337)/1000;//メートル表示変換
                    TPCC[DTPC_ok].y = -DTPC[i] * ((tate_point_curr[i].y - 241.696) / 615.458)/1000;
                    TPCC[DTPC_ok].z = DTPC[i]/1000;
                    tate_point_curr[DTPC_ok] = tate_point_curr[i];
	    	        cv::circle(img_tate, tate_point_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
                    //テンプレート作成----------------------------------------------------------------------------------------
                    //std::cout <<"縦線中点画像クロッププログラム["<<i<<"]"<< std::endl;//最初のフレーム
                    cv::Rect roi2(cv::Point(tate_point_curr[i].x-template_size,tate_point_curr[i].y-template_size), cv::Size(template_size*2, template_size*2));//縦線中点を中心とした16☓16pixelの画像を切り取る
                    TPCC_Templ[DTPC_ok] = img_src(roi2); // 切り出し画像
                    cv::rectangle(img_tate, roi2,cv::Scalar(255, 255, 255), 2);//テンプレート位置
                    //cv::rectangle(img_master_temp, cv::Point(tate_point_curr[i].x-template_size,tate_point_curr[i].y+template_size), 
                    //cv::Point(tate_point_curr[i].x+template_size,tate_point_curr[i].y-template_size), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写(白)
                    //std::cout <<"縦線中点の画像座標(DTPC_ok)["<<DTPC_ok<<"]="<<tate_point_curr[DTPC_ok]<< std::endl;//縦線中点の座標(範囲制限後)
                    DTPC_ok=DTPC_ok+1;//Depth取得可能+テンプレ制限範囲内の個数をカウント
                }
            }
        }
    }
    tate_point_curr.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_c)
    tate_point_prev.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_c)
    TPCC.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_c)
    TPCP.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_prev)
    Est_tate_point.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_prev)
    Est_tate_pixel.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_prev)

    reset = false;//if文切り替え
    //std::cout <<"初回検出プログラム終了"<< std::endl;

   //ロボット指令プログラム--------------------------------------------------------------------------------------------
   //別にここの位置に書く必要はない
    robot_odometry=*msg;
    /*//廊下直進動作----------------------------------------------------------------------------------------------------------------
      std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      //直進動作
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);//実行指令値
        robot_velocity.angular.z = (Des_RobotTH-Act_RobotTH)*2.2;

        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.linear.x  = VX;} 
        robot_velocity.angular.z = 0.0;
        if(Des_RobotX>=LX){//停止命令
          X_25=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      }*/
    



    //feedback制御の動作ここから-------------------------------------------------------------------------------------------------------
      robot_velocity.linear.x = 0.0;
      robot_velocity.angular.z = 0.0;
      

      if(decide_subgoal == true){
        int max_img_depth = img_depth.at<float>(239,0);
        int max_pixel_x = 0;
        float maximum_depth_x;
        float maximum_depth_y;
        float x_k;
        float y_k;
        for(int i = 1;i<640;i++){
          if(max_img_depth < img_depth.at<float>(239,i)){
            max_img_depth = img_depth.at<float>(239,i);
            max_pixel_x = i;
          }
        }
        std::cout<<"-----------------------------------max_img_depth======"<<max_img_depth<<"-----------------------------------------"<<std::endl;
        std::cout<<"-----------------------------------max_pixel_x======"<<max_pixel_x<<"-----------------------------------------"<<std::endl;
        // printf("max_img_depth = %f\n",max_img_depth);
        // printf("一番大きいpixel = %f\n",max_pixel_x);
        maximum_depth_x = -2200*(2*max_pixel_x-640)/(2*615.337);
        maximum_depth_y = -max_img_depth*(2*239-480)/(2*615.458);
        printf("maximum_depth_x = %f\n",maximum_depth_x);
        printf("maximum_depth_y = %f\n",maximum_depth_y);
        x_k = max_img_depth;
        y_k = -maximum_depth_x;
        printf("x_k = %f\n",x_k);
        printf("y_k = %f\n",y_k);
        Goal_RobotX = maximum_depth_x;
        Goal_RobotY = maximum_depth_y;
        distance_to_goal = sqrt(pow(Goal_RobotX,2)+pow(Goal_RobotY,2));
        Goal_RobotTH = atan2(Goal_RobotX,2200);
        printf("distance_to_goal = %f\n",distance_to_goal);
        printf("Goal_RobotTH = %f\n",Goal_RobotTH);
        Goal_RobotX_txt<<Goal_RobotX/1000<<"\n";

        decide_subgoal = false;


        // subgoal(img_depth);
        // Goal_RobotX = subgoalX;
        // Goal_RobotY = subgoalY;
        // distance_to_goal = sqrt(pow(Goal_RobotX,2)+pow(Goal_RobotY,2));
        // Goal_RobotTH = atan2(Goal_RobotY,1000);
        // decide_subgoal = false;
      }
      
     
      // subgoal(img_depth);
      // Goal_RobotX = subgoalX;
      // Goal_RobotY = subgoalY;
      // printf("subgoalX = %f\n",subgoalX);
      // printf("subgoalY = %f\n",subgoalY);
      // distance_to_goal = sqrt(pow(Goal_RobotX,2)+pow(Goal_RobotY,2));
      // Goal_RobotTH = atan2(Goal_RobotY,Goal_RobotX);
      // printf("distance_to_goal = %f\n",distance_to_goal);
      // printf("Goal_RobotTH = %f\n",Goal_RobotTH);
      printf("rotation_goal = %d\n",rotation_goal);
      printf("straight_goal = %d\n",straight_goal);
      
      
      if(rotation_goal == false){
        robot_velocity.angular.z = 100000*(Goal_RobotTH - Est_RobotTH);
        robot_velocity.linear.x = 0.0;
        if(robot_velocity.angular.z >= 0.1){
          robot_velocity.angular.z = 0.1;
          robot_velocity.linear.x = 0.0;
        }else if(robot_velocity.angular.z <= -0.1){
          robot_velocity.angular.z = -0.1;
          robot_velocity.linear.x = 0.0;
        }
        if(abs(Goal_RobotTH) <= abs(Est_RobotTH)){
          rotation_goal = true;
          usleep(7*100000);
        }
      }
    // rotation_goal = true;
    if(rotation_goal == true){
      if(straight_goal == false){
        straight_goal = true;
        straight_begin = ros::Time::now();
      }
        //printf("------------------------------------------------------------------------------------------------------------");
        // subgoal(img_depth);
        // Goal_RobotX = subgoalX;
        // Goal_RobotY = subgoalY;
        // distance_to_goal = sqrt(pow(Goal_RobotX,2)+pow(Goal_RobotY,2));
        // Goal_RobotTH = atan2(Goal_RobotY,Goal_RobotX);

        //robot_velocity.linear.x = 10000*(sqrt(pow(Goal_RobotX-Est_RobotX,2)+pow(Goal_RobotY-Est_RobotY,2)));

        // if(robot_velocity.linear.x >= 0.25){
        //   robot_velocity.linear.x = 0.25;
        // }

        if(ros::Time::now().toSec() - straight_begin.toSec()<=20){
          robot_velocity.linear.x = 0.25;
          // robot_velocity.angular.z = 0.055;//左回避用補正プログラム
          // robot_velocity.angular.z = 0.0;
          // robot_velocity.angular.z = -0.04;//右回避用補正プログラム

        }

        // if(ros::Time::now().toSec() - straight_begin.toSec() < 4){
        //   // decide_subgoal = true;
        //   rotation_goal = false;
        // }

        // if(Goal_RobotX <= Est_RobotX){
        //   decide_subgoal = true;
        //   rotation_goal = false;
        // }
      }
        printf("robot_velocity.linear.x = %f\n",robot_velocity.linear.x);
        printf("robot_velocity.angular.z = %f\n",robot_velocity.angular.z);
        pub.publish(robot_velocity);
      




    //feedback制御の動作ここまで--------------------------------------------------------------------------------------------------------




    //廊下動作1-----------------------------------------------------------------------------------------------------------------
      // //std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      // //直進動作
      // if(X_25==false&&TH_90==false&&Y_05==false){
      //   robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);//実行指令値
      //   //robot_velocity.linear.x  = VX;//実行指令値

      //   //std::cout << "robot_velocity.linear.x=" <<robot_velocity.linear.x<< std::endl;
      //   robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;
      //   pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      //   if(Act_RobotV<=0.001){robot_velocity.linear.x=0;}//指令値の遅れを考慮(推定に使用する指令値)
      //   else{robot_velocity.linear.x  = VX;} 
      //   //robot_velocity.linear.x  = VX;


      //   robot_velocity.angular.z = 0.0;
      //   if(Des_RobotX>=LX){//停止命令
      //     X_25=true;
      //     robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      //     robot_velocity.angular.z = 0.0; // 回転速度ωの初期化
      //     pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      //     usleep(7*100000);//0.5秒ストップ(マイクロ秒)
      //   }
      // }
      // //一時停止
      // //回転動作(半時計回り)
      // else if(X_25==true&&TH_90==false&&Y_05==false){
      //   robot_velocity.linear.x  =  0;
      //   robot_velocity.angular.z  =  THZ+(Des_RobotTH-Act_RobotTH)*2.2;//実行指令値
      //   pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      //   robot_velocity.linear.x  =  0.0;
      //   robot_velocity.angular.z  =  THZ; 
      //   if(Act_RobotTH<=0.001){robot_velocity.angular.z  =  0;
      //   }//指令値の遅れを考慮(推定に使用する指令値)
      //   else{robot_velocity.angular.z  =  THZ;} 
      //   //robot_velocity.angular.z  =  THZ; 

      //   if(Des_RobotTH>=3.141592653/omegaZ){//廊下
      //     TH_90=true;
      //     robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      //     robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      //     pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      //     usleep(7*100000);//0.5秒ストップ(マイクロ秒)
      //   }
      // }
      // //直進1回目
      // else if(X_25==true&&TH_90==true&&Y_05==false){
      //   robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);
      //   robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;
      //   std::cout << "robot_velocity.linear.x=" <<robot_velocity.linear.x<< std::endl;
      //   pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      //   if(Act_RobotV<=0.001){robot_velocity.linear.x=0;}
      //   else{robot_velocity.linear.x  = VX;} // 並進速度vの初期化
      //   //robot_velocity.linear.x  = VX;

      //   robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      //   if(Des_RobotY>=LY){
      //     Y_05=true;
      //     robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      //     robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      //     pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      //     usleep(7*100000);//0.5秒ストップ(マイクロ秒)
      //   }
      // }
      // //else if(X_25==true&&TH_90==true&&Y_05==false){
      // //  robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);
      // //  robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;
      // //  pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      // //  if(Act_RobotV<=0.00001){robot_velocity.linear.x=0;}
      // //  else{robot_velocity.linear.x  = VX;} // 並進速度vの初期化
      // //  robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      // //  if(Des_RobotX>=LY){
      // //    Y_05=true;
      // //    robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      // //    robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      // //    pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      // //    usleep(7*100000);//0.5秒ストップ(マイクロ秒)
      // //  }
      // //}

      // else if(X_25==true&&TH_90==true&&Y_05==true){
      //   robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      //   robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      //   pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      // }
      //廊下動作1ここまで-----------------------------------------------------------------------------------------------------------------

    Act_RobotV=robot_odometry.twist.twist.linear.x+robot_odometry.twist.twist.linear.y;//速度ベクトルの合成
    //std::cout << "Act_RobotV=" <<Act_RobotV<< std::endl;

    Des_RobotV=robot_velocity.linear.x+robot_velocity.linear.y;//速度ベクトルの合成
    //std::cout << "Des_RobotV=" <<Des_RobotV<< std::endl;

    //この下にカルマンフィルタを実装する(できたら別関数にしたい)
    //カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
    if(kaisu==0){
      DES_Robot = cv::Mat_<double>::zeros(3, 1);//目標指令状態
      ACT_Robot = cv::Mat_<double>::zeros(3, 1);//雑音の影響を考慮した実際の状態
      EST_Robot = cv::Mat_<double>::zeros(3, 1);//推定状態

      Cov = (cv::Mat_<double>(3,3) << //共分散Q
        1e-10, 0,      0,
        0,     1e-10,  0,
        0,     0,      1e-10);
      I = cv::Mat_<double>::eye(3, 3);//単位行列
      //std::cout <<"初期設定"<< std::endl;
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------

    MTPC.resize(1000);//Matching_Tate_Pixel_Curr=MTPC定義(ここの大きさを超えるとエラーになる)
    //テンプレート範囲予測とテンプレートマッチング----------------------------------------------------------------------
    //テンプレートマッチングは1つ前のテンプレートを使用(テンプレート:t+n-1,マッチング画像:t+n)
  if(time0 != false){
    if (reset == false) {//初回以降動作
      //ロボットの動きから特徴点の運動復元を行う(特徴点の状態方程式を使用)
      //std::cout <<"二回目動作:DTPP_ok="<<DTPP_ok<< std::endl;

      matchT_curr=0;
      for(int i=0;i<DTPP_ok;i++){
        //std::cout <<"\n"<< std::endl;
        //std::cout <<"TPCP["<<i<<"]="<<TPCP[i]<< std::endl;
        //std::cout <<"robot_odometry.twist.twist.angular.z="<<robot_odometry.twist.twist.angular.z<< std::endl;
        //std::cout <<"realsec="<<realsec<< std::endl;
        //std::cout <<"Act_RobotV="<<Act_RobotV<< std::endl;

        Est_tate_point[i].x=-TPCP[i].x+robot_odometry.twist.twist.angular.z*realsec*TPCP[i].z+Act_RobotV*sin(-Act_RobotTH)*realsec;
        Est_tate_point[i].y=-TPCP[i].y;
        Est_tate_point[i].z=TPCP[i].z-robot_odometry.twist.twist.angular.z*realsec*-TPCP[i].x+Act_RobotV*cos(-Act_RobotTH)*realsec;

        Est_tate_pixel[i].x=324.473+(Est_tate_point[i].x/Est_tate_point[i].z)*615.337;
        Est_tate_pixel[i].y=241.696+(Est_tate_point[i].y/Est_tate_point[i].z)*615.458;

        //cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x,Est_tate_pixel[i].y), 6, Scalar(0,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
        cv::line(img_tate,cv::Point(tate_point_prev[i].x,tate_point_prev[i].y),cv::Point(Est_tate_pixel[i].x,Est_tate_pixel[i].y),cv::Scalar(0,0,255), 1, cv::LINE_AA);

        //std::cout <<"Est_tate_point["<<i<<"]="<<Est_tate_point[i]<< std::endl;//予測カメラ座標
        //std::cout <<"Est_tate_pixel["<<i<<"]="<<Est_tate_pixel[i]<< std::endl;//予測画像座標

        //求めた運動復元結果からテンプレートマッチングの予測範囲を作る(とりあえずタテヨコ2倍)
        //std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;

        //予測範囲が全て画面内の時
        if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
          //std::cout << "予測範囲が全て画面内の時["<<i<<"]"<< std::endl;
          cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropx), cv::Size(cropx*2, cropx*2));//線の中点を中心とした線の画像を切り取る
          EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
          cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          //cv::imshow("EST_tate_scope", EST_tate_scope[i]);//黄色の特徴点を中心としたクロップ画像
        }
        //左側
        else if(0<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<cropx){
          //左上(xとyどちらもはみでる)
          if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
            //std::cout << "左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,0), cv::Size(Est_tate_pixel[i].x+cropx, Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
           cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //左側(xははみ出ない)
          else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
            //std::cout << "左側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,Est_tate_pixel[i].y-cropy), cv::Size(Est_tate_pixel[i].x+cropx, cropx*2));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //左下(xとyどちらもはみでる)
          else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
            //std::cout << "左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,Est_tate_pixel[i].y-cropy), cv::Size(Est_tate_pixel[i].x+cropx, 480-Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
        }
        //上側(yははみ出ない)
        else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
          //std::cout << "上側(yははみ出ない)["<<i<<"]"<<std::endl;
          cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,0), cv::Size(cropx*2, Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
          EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
          cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
        }
        //右側
        else if(640-cropx<Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640){
          //右上(xとyどちらもはみでる)
          if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
            //std::cout << "右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,0), cv::Size(640-Est_tate_pixel[i].x+cropx, Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //右側(xははみ出ない)
          else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
            //std::cout << "右側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropy), cv::Size(640-Est_tate_pixel[i].x+cropx, cropy*2));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //右下(xとyどちらもはみでる)
          else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
            //std::cout << "右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropy), cv::Size(640-Est_tate_pixel[i].x+cropx, 480-Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
        }
        //下側(yははみ出ない)
        else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&480-cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<480){
          //std::cout << "下側(yははみ出ない)["<<i<<"]"<<std::endl;
          cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropy), cv::Size(cropx*2, 480-Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
          EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
          cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
        }
        else{
          //std::cout << "画面外["<<i<<"]"<< std::endl;
        }

        //std::cout <<"TPCP_Templ[i].cols="<<TPCP_Templ[i].cols<<"TPCP_Templ[i].rows="<<TPCP_Templ[i].rows<< std::endl;//最初のフレーム
        //std::cout <<"EST_tate_scope[i].cols="<<EST_tate_scope[i].cols<<"EST_tate_scope[i].rows="<<EST_tate_scope[i].rows<< std::endl;//最初のフレーム

        //予測範囲に対しテンプレートマッチングを行う
        //std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
        TPCP_Templ[i].copyTo(img_template1); // 切り出し画像
        //cv::imshow("win_TPCP_Templ", TPCP_Templ[i]);//黄色の特徴点を中心としたクロップ画像

        cv::Mat img_minmax1;
        // テンプレートマッチング
        cv::matchTemplate(EST_tate_scope[i], img_template1, img_minmax1, cv::TM_CCOEFF_NORMED);//正規化相互相関(ZNCC)
        cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
        //std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
        //std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
        if(0.8<max_val1[i]){//最小値がしきい値以下なら表示
          //予測範囲が全て画面内の時
          if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
            //std::cout << "マッチング:全て画面内の時["<<i<<"]"<< std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像(Matching_Tate_Templ_curr=MTTC)
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
          }
             //左側
          else if(0<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<cropx){
            //左上(xとyどちらもはみでる)
            if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
              //std::cout << "マッチング:左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
              MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
              //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
              matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //左側(xははみ出ない)
            else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
              //std::cout << "マッチング:左側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //左下(xとyどちらもはみでる)
            else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
              //std::cout << "マッチング:左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
          }
          //上側(yははみ出ない)
          else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
            //std::cout << "マッチング:上側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
          }
          //右側
          else if(640-cropx<Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640){
            //右上(xとyどちらもはみでる)
            if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
              //std::cout << "マッチング:右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //右側(xははみ出ない)
            else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
              //std::cout << "マッチング:右側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //右下(xとyどちらもはみでる)
            else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
              //std::cout << "マッチング:右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
          }
          //下側(yははみ出ない)
          else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&480-cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<480){
            //std::cout << "マッチング:下側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            //std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
          }
          else{
            //std::cout << "画面外["<<i<<"]"<< std::endl;
          }
        }//if(min_val1[i]<max_val1[i]*0.05)→end(テンプレートマッチング)
      }//for(int i=0;i<DTPP_ok;i++)→end (範囲予測+テンプレートマッチング)
      //std::cout <<"matchT_curr="<<matchT_curr<< std::endl;
      MTPC.resize(matchT_curr);//Depth取得可能数でリサイズ(運動復元画像座標)
      MT_curr_pixel.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心画像座標)
      MT_curr_camera.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)
      //std::cout <<"test"<< std::endl;

      //マッチングしたテンプレートをマッチテンプレートとしてキープする
      //マッチテンプレートのDepthが取得不能な点を削除
      DMT_curr_ok=0;
      for (int i = 0; i < matchT_curr; i++) {
        DMT_curr[i] = img_depth.at<float>(cv::Point(MTPC[i].x,MTPC[i].y));//DMT_curr=Depth_MTPC
        //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
        if(DMT_curr[i]>0.001&&DMT_curr[i]<10000){
          MT_curr_Templ[DMT_curr_ok] = MTTC[i];//Depth取得可能なマッチテンプレート
          MT_curr_pixel[DMT_curr_ok] = MTPC[i];//Depth取得可能なマッチング中心画像座標
          MT_curr_camera[DMT_curr_ok].x = -DMT_curr[i] * ((MTPC[i].x - 324.473) / 615.337)/1000;//カメラ座標変換
          MT_curr_camera[DMT_curr_ok].y = -DMT_curr[i] * ((MTPC[i].y - 241.696) / 615.458)/1000;
          MT_curr_camera[DMT_curr_ok].z = DMT_curr[i]/1000;
          //std::cout <<"MT_curr_pixel["<<DMT_curr_ok<<"]="<<MT_curr_pixel[DMT_curr_ok]<< std::endl;
          //std::cout <<"MT_curr_camera["<<DMT_curr_ok<<"]="<<MT_curr_camera[DMT_curr_ok]<< std::endl;
          //std::cout <<"MT_curr_Templ["<<DMT_curr_ok<<"].cols="<<MT_curr_Templ[DMT_curr_ok].cols<<"MT_curr_Templ["<<DMT_curr_ok<<"].rows="<<MT_curr_Templ[DMT_curr_ok].rows<< std::endl;//最初のフレーム
          DMT_curr_ok=DMT_curr_ok+1;//Depthが取得可能なマッチテンプレート数
          //DMT_prev_ok1=DMT_prev_ok1+1;//新規テンプレート数
        }
		  }
      //std::cout <<"新規テンプレート数:DMT_curr_ok="<<DMT_curr_ok<< std::endl;
      //類似するテンプレートを削除する
      int notsimil=0;
      for (int i = 0; i < DMT_curr_ok; i++) {
        int similar=0;
        for (int j = i+1; j < DMT_curr_ok; j++) {
          length=sqrt((MT_curr_pixel[i].x-MT_curr_pixel[j].x)*(MT_curr_pixel[i].x-MT_curr_pixel[j].x)
            +(MT_curr_pixel[i].y-MT_curr_pixel[j].y)*(MT_curr_pixel[i].y-MT_curr_pixel[j].y));
          if(template_size*2>length){
            similar=similar+1;
          }
        }
        //類似するテンプレートを繰り上げ削除
        if(similar==0){
          MT_curr_Templ[notsimil]=MT_curr_Templ[i];
          MT_curr_pixel[notsimil]=MT_curr_pixel[i];
          MT_curr_camera[notsimil]=MT_curr_camera[i];
          notsimil=notsimil+1;
        }
      }
      DMT_curr_ok=notsimil;
      //std::cout <<"新規テンプレート数(修正後):DMT_curr_ok="<<DMT_curr_ok<< std::endl;

      MT_curr_pixel.resize(DMT_curr_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
      MT_curr_camera.resize(DMT_curr_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)

      Est_MT_point.resize(1000);//配列初期設定
      Est_MT_pixel.resize(1000);//配列初期設定

      //世界座標推定(マーカー観測時)------------------------------------------------------------------
      //テンプレートに一番近いマーカーの座標を使用する(カメラ観測座標で)
      //(マーカー観測が不可能な場合は3回目動作後に推定する)
      MT_curr_world.resize(1000);//初期設定
      if(markerIds.size() > 0){
        //std::cout <<"マーカー観測可能"<< std::endl;
        double minlengh=1000000;
        int minMC;
        for (int i = 0; i < DMT_curr_ok; i++) {
          for(int j = 0; j < markerIds.size(); j++){
            length=sqrt((MT_curr_camera[i].x-MC_point[j][0])*(MT_curr_camera[i].x-MC_point[j][0])
            +(MT_curr_camera[i].y-MC_point[j][1])*(MT_curr_camera[i].y-MC_point[j][1])
            +(MT_curr_camera[i].z-MC_point[j][2])*(MT_curr_camera[i].z-MC_point[j][2]));
            if(minlengh>length){
              minMC=j;
              minlengh=length;
            }
          }
          MT_curr_world[i].x=MarkerW[markerIds.at(minMC)].at<float>(0)+(MT_curr_camera[i].x-MC_point[markerIds.at(minMC)][0])*cos(-Est_RobotTH)-(MT_curr_camera[i].z-MC_point[markerIds.at(minMC)][2])*sin(-Est_RobotTH);
          MT_curr_world[i].y=MarkerW[markerIds.at(minMC)].at<float>(1)+(MT_curr_camera[i].y-MC_point[markerIds.at(minMC)][1]);
          MT_curr_world[i].z=MarkerW[markerIds.at(minMC)].at<float>(2)+(MT_curr_camera[i].x-MC_point[markerIds.at(minMC)][0])*sin(-Est_RobotTH)+(MT_curr_camera[i].z-MC_point[markerIds.at(minMC)][2])*cos(-Est_RobotTH);

          MP_kaisu<<ALLrealsec<<"\n";
          MP_worldy<<MT_curr_world[i].x<<"\n";
          MP_worldx<<MT_curr_world[i].z<<"\n";
          MC_worldy<<MarkerW[markerIds.at(minMC)].at<float>(0)<<"\n";
          MC_worldx<<MarkerW[markerIds.at(minMC)].at<float>(2)<<"\n";
          MP_cameray<<MT_curr_camera[i].x<<"\n";
          MP_camerax<<MT_curr_camera[i].z<<"\n";
          MC_cameray<<MC_point[markerIds.at(minMC)][0]<<"\n";
          MC_camerax<<MC_point[markerIds.at(minMC)][2]<<"\n";

          //std::cout <<"MC_point["<<markerIds.at(minMC)<<"]=("<<MC_point[markerIds.at(minMC)][0]<<", "<<MC_point[markerIds.at(minMC)][1]<<", "<<MC_point[markerIds.at(minMC)][2]<<")"<< std::endl;
          //std::cout <<"MT_curr_camera["<<i<<"]="<<MT_curr_camera[i]<< std::endl;

          //std::cout <<"MarkerW["<<markerIds.at(minMC)<<"]=("<<MarkerW[markerIds.at(minMC)].at<float>(0)<<", "<<MarkerW[markerIds.at(minMC)].at<float>(1)<<", "<<MarkerW[markerIds.at(minMC)].at<float>(2)<<")"<< std::endl;
          //std::cout <<"MT_curr_world["<<i<<"]="<<MT_curr_world[i]<< std::endl;
        }
        MT_curr_world.resize(DMT_curr_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
        //for (int i = 0; i < DMT_curr_ok; i++) {
        //  std::cout <<"新規テンプレートの世界座標:MT_curr_world["<<i<<"]="<<MT_curr_world[i]<< std::endl;
        //}
      }

      //3回目以降動作
      if(Tracking == true){
        //std::cout <<"3回目以降動作-------------------------------------------------------"<< std::endl;
        //ロボットの動きから一つ前のマッチング座標の運動復元を行う(特徴点の状態方程式を使用)
        EST_MT_ok=0;
        for(int i=0;i<DMT_prev_ok;i++){
          //std::cout <<"\n"<< std::endl;
          //std::cout <<"MT_prev_world["<<i<<"]="<<MT_prev_world[i]<< std::endl;
          //std::cout <<"MT_prev_camera["<<i<<"]="<<MT_prev_camera[i]<< std::endl;
          //std::cout <<"MT_prev_pixel["<<i<<"]="<<MT_prev_pixel[i]<< std::endl;
          //std::cout <<"robot_odometry.twist.twist.angular.z="<<robot_odometry.twist.twist.angular.z<< std::endl;
          //std::cout <<"realsec="<<realsec<< std::endl;
          //std::cout <<"Act_RobotV="<<Act_RobotV<< std::endl;

          Est_MT_point[i].x=-MT_prev_camera[i].x+robot_odometry.twist.twist.angular.z*realsec*MT_prev_camera[i].z+Act_RobotV*sin(-Act_RobotTH)*realsec;
          Est_MT_point[i].y=-MT_prev_camera[i].y;
          Est_MT_point[i].z=MT_prev_camera[i].z-robot_odometry.twist.twist.angular.z*realsec*-MT_prev_camera[i].x+Act_RobotV*cos(-Act_RobotTH)*realsec;

          Est_MT_pixel[i].x=324.473+(Est_MT_point[i].x/Est_MT_point[i].z)*615.337;
          Est_MT_pixel[i].y=241.696+(Est_MT_point[i].y/Est_MT_point[i].z)*615.458;

          //cv::circle(img_tate, cv::Point(Est_pixel[i].x,Est_pixel[i].y), 6, Scalar(0,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
          //一つ前のマッチ画像座標と次の推定画像座標
          cv::line(img_tate,cv::Point(MT_prev_pixel[i].x,MT_prev_pixel[i].y),cv::Point(Est_MT_pixel[i].x,Est_MT_pixel[i].y),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)

          //std::cout <<"Est_MT_point["<<i<<"]=\n"<<Est_MT_point[i]<< std::endl;//予測カメラ座標
          //std::cout <<"Est_MT_pixel["<<i<<"]=\n"<<Est_MT_pixel[i]<< std::endl;//予測画像座標

          //求めた運動復元結果からテンプレートマッチングの予測範囲を作る(とりあえずタテヨコ2倍)
          //ここで予測点が画面外に行ったらそのテンプレートを削除する
          //std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;
          //MTcoix[i]=Est_MT_pixel[i].x+template_size;//予測範囲の中心座標
          //MTcoiy[i]=Est_MT_pixel[i].y+template_size;//予測範囲の中心座標
          //予測範囲が全て画面内の時
          if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
            //std::cout << "予測範囲が全て画面内の時["<<i<<"]"<< std::endl;
            cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropx), cv::Size(cropx*2, cropx*2));//線の中点を中心とした線の画像を切り取る
            EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
            MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
            MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
            Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
            Est_MT_point[EST_MT_ok] = Est_MT_point[i];
            MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
            MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
            MT_prev_world[EST_MT_ok]=MT_prev_world[i];
            MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
           // cv::imshow("EST_MT_scope", EST_MT_scope[EST_MT_ok]);//黄色の特徴点を中心としたクロップ画像
            EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
          }
           //左側
          else if(0<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<cropx){
            //左上(xとyどちらもはみでる)
            if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
              //std::cout << "左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,0), cv::Size(Est_MT_pixel[i].x+cropx, Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_world[EST_MT_ok]=MT_prev_world[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //左側(xははみ出ない)
            else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
              //std::cout << "左側(xははみ出ない)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,Est_MT_pixel[i].y-cropy), cv::Size(Est_MT_pixel[i].x+cropx, cropx*2));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_world[EST_MT_ok]=MT_prev_world[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //左下(xとyどちらもはみでる)
            else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
              //std::cout << "左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,Est_MT_pixel[i].y-cropy), cv::Size(Est_MT_pixel[i].x+cropx, 480-Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_world[EST_MT_ok]=MT_prev_world[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
          }
          //上側(yははみ出ない)
          else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
            //std::cout << "上側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,0), cv::Size(cropx*2, Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
            MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
            MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
            Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
            Est_MT_point[EST_MT_ok] = Est_MT_point[i];
            MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
            MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
            MT_prev_world[EST_MT_ok]=MT_prev_world[i];
            MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
          }
          //右側
          else if(640-cropx<Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640){
            //右上(xとyどちらもはみでる)
            if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
              //std::cout << "右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,0), cv::Size(640-Est_MT_pixel[i].x+cropx, Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_world[EST_MT_ok]=MT_prev_world[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //右側(xははみ出ない)
            else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
              //std::cout << "右側(xははみ出ない)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropy), cv::Size(640-Est_MT_pixel[i].x+cropx, cropy*2));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_world[EST_MT_ok]=MT_prev_world[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //右下(xとyどちらもはみでる)
            else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
              //std::cout << "右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropy), cv::Size(640-Est_MT_pixel[i].x+cropx, 480-Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_world[EST_MT_ok]=MT_prev_world[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
          }
          //下側(yははみ出ない)
          else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&480-cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<480){
            //std::cout << "下側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropy), cv::Size(cropx*2, 480-Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
            MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
            MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
            Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
            Est_MT_point[EST_MT_ok] = Est_MT_point[i];
            MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
            MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
            MT_prev_world[EST_MT_ok]=MT_prev_world[i];
            MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
          }
          else{
            //std::cout << "画面外["<<i<<"]"<< std::endl;
          }
        }//for(int i=0;i<DMT_prev_ok;i++)→end (範囲予測+テンプレートマッチング)
        //std::cout << "EST_MT_ok="<<EST_MT_ok<< std::endl;
        Est_MT_pixel.resize(EST_MT_ok);//リサイズ
        Est_MT_point.resize(EST_MT_ok);//リサイズ
        MT_prev_pixel.resize(EST_MT_ok);//リサイズ
        MT_prev_camera.resize(EST_MT_ok);//リサイズ
        MT_prev_world.resize(EST_MT_ok);//リサイズ
        MTPP.resize(1000);//配列初期設定

        //テンプレートマッチングが原因っぽい
        //予測範囲に対しテンプレートマッチングを行う
        matchT_prev=0;
        for(int i=0;i<EST_MT_ok;i++){
          //std::cout <<"\n"<< std::endl;
          //std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
          //MT_prev_Templ[i].copyTo(img_template1); // 切り出し画像(多分これが原因)
          img_template1 = MT_prev_Templ[i].clone();

          //テンプレートの拡大-----------------------------------------------
          //double scale=sqrt(Est_MT_point[i].x*Est_MT_point[i].x+Est_MT_point[i].y*Est_MT_point[i].y+Est_MT_point[i].z*Est_MT_point[i].z)
          ///sqrt(MT_prev_camera[i].x*MT_prev_camera[i].x+MT_prev_camera[i].y*MT_prev_camera[i].y+MT_prev_camera[i].z*MT_prev_camera[i].z);
          //cv::resize(img_template1, img_template1, cv::Size(), scale, scale);

          cv::Mat img_minmax1;
          // テンプレートマッチング
          cv::matchTemplate(EST_MT_scope[i], img_template1, img_minmax1, cv::TM_CCOEFF_NORMED);//正規化相互相関(ZNCC)
          cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
          //std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
          //std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;

          if(0.8<max_val1[i]){//最小値がしきい値以下なら表示
            //予測範囲が全て画面内の時
            if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
              //std::cout << "マッチング:全て画面内の時["<<i<<"]"<< std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

              //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
              MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
              MT_prev_camera[matchT_prev]=MT_prev_camera[i];
              MT_prev_world[matchT_prev]=MT_prev_world[i];
              MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
              //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
              matchT_prev=matchT_prev+1;//マッチングの中心座標個数
            }
            //左側
            else if(0<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<cropx){
              //左上(xとyどちらもはみでる)
              if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
                //std::cout << "マッチング:左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_world[matchT_prev]=MT_prev_world[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              }
              //左側(xははみ出ない)
              else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
                //std::cout << "マッチング:左側(xははみ出ない)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_world[matchT_prev]=MT_prev_world[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              }
              //左下(xとyどちらもはみでる)
              else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
                //std::cout << "マッチング:左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_world[matchT_prev]=MT_prev_world[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              }
            }
            //上側(yははみ出る)
            else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
              //std::cout << "マッチング:上側(yははみ出る)["<<i<<"]"<<std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPP[matchT_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

              //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
              MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
              MT_prev_camera[matchT_prev]=MT_prev_camera[i];
              MT_prev_world[matchT_prev]=MT_prev_world[i];
              MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
             // std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
              matchT_prev=matchT_prev+1;//マッチングの中心座標個数
            }
            //右側
            else if(640-cropx<Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640){
              //右上(xとyどちらもはみでる)
              if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
                //std::cout << "マッチング:右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_world[matchT_prev]=MT_prev_world[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              }
              //右側(xははみ出ない)
              else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
                //std::cout << "マッチング:右側(xははみ出ない)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_world[matchT_prev]=MT_prev_world[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              }
              //右下(xとyどちらもはみでる)
              else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
                //std::cout << "マッチング:右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_world[matchT_prev]=MT_prev_world[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              }
            }
            //下側(yははみ出ない)
            else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&480-cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<480){
              //std::cout << "マッチング:下側(yははみ出ない)["<<i<<"]"<<std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

              //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
              MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
              MT_prev_camera[matchT_prev]=MT_prev_camera[i];
              MT_prev_world[matchT_prev]=MT_prev_world[i];
              MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
              //std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
              matchT_prev=matchT_prev+1;//マッチングの中心座標個数
            }
            else{
              //std::cout << "画面外["<<i<<"]"<< std::endl;
            }
          }//if(min_val1[i]<max_val1[i]*0.05)→end(テンプレートマッチング)
          else{
              //std::cout << "マッチしない["<<i<<"]((0.99<max_val1["<<i<<"])="<<0.99<<"<"<<max_val1[i]<< std::endl;
          }
        }//for(int i=0;i<DMT_prev_ok;i++)→end (範囲予測+テンプレートマッチング)

        //std::cout <<"matchT_prev="<<matchT_prev<< std::endl;
        MTPP.resize(matchT_prev);//Depth取得可能数でリサイズ(運動復元画像座標)
        MT_prev_pixel.resize(matchT_prev);//リサイズ
        MT_prev_camera.resize(matchT_prev);//リサイズ
        MT_prev_world.resize(matchT_prev);//リサイズ
        MT_curr2_pixel.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心画像座標)
        MT_curr2_camera.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)
        MT_curr2_world.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)

        //マッチングしたテンプレートをマッチテンプレートとしてキープする
        //マッチテンプレートのDepthが取得不能な点を削除
        DMT_curr2_ok=0;
        for (int i = 0; i < matchT_prev; i++) {
          DMT_curr2[i] = img_depth.at<float>(cv::Point(MTPP[i].x,MTPP[i].y));//DMP_prev=Depth_MTPP
          //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
          if(DMT_curr2[i]>0.001&&DMT_curr2[i]<10000){
            MT_curr2_Templ[DMT_curr2_ok] = MTTP[i];//Depth取得可能なマッチテンプレート
            MT_curr2_pixel[DMT_curr2_ok] = MTPP[i];//Depth取得可能なマッチング中心画像座標
            MT_curr2_camera[DMT_curr2_ok].x = -DMT_curr2[i] * ((MTPP[i].x - 324.473) / 615.337)/1000;//カメラ座標変換
            MT_curr2_camera[DMT_curr2_ok].y = -DMT_curr2[i] * ((MTPP[i].y - 241.696) / 615.458)/1000;
            MT_curr2_camera[DMT_curr2_ok].z = DMT_curr2[i]/1000;
            MT_curr2_world[DMT_curr2_ok] = MT_prev_world[i];//Depth取得可能なマッチング中心世界座標
            //std::cout <<"MT_curr2_pixel["<<DMT_curr2_ok<<"]="<<MT_curr2_pixel[DMT_curr2_ok]<< std::endl;
            //std::cout <<"MT_curr2_camera["<<DMT_curr2_ok<<"]="<<MT_curr2_camera[DMT_curr2_ok]<< std::endl;
            //std::cout <<"MT_curr2_world["<<DMT_curr2_ok<<"]="<<MT_curr2_world[DMT_curr2_ok]<< std::endl;
            DMT_curr2_ok=DMT_curr2_ok+1;//Depthが取得可能な全マッチテンプレート数
            //DMT_prev_ok3=DMT_prev_ok3+1;//保持テンプレート数
          }
		    }

        /*std::cout <<"新規テンプレート数(2回目動作結果):DMT_curr_ok="<<DMT_curr_ok<< std::endl;
        for (int i = 0; i < DMT_curr_ok; i++) {
          std::cout <<"MT_curr_pixel["<<i<<"]="<<MT_curr_pixel[i]<< std::endl;
          //std::cout <<"MT_curr_camera["<<i<<"]="<<MT_curr_camera[i]<< std::endl;
        }
        std::cout <<"3回目テンプレート数:DMT_curr2_ok="<<DMT_curr2_ok<< std::endl;
        for (int i = 0; i < DMT_curr2_ok; i++) {
          std::cout <<"MT_curr2_pixel["<<i<<"]="<<MT_curr2_pixel[i]<< std::endl;
          //std::cout <<"MT_curr2_camera["<<i<<"]="<<MT_curr2_camera[i]<< std::endl;
        }
        std::cout <<"一つ前のテンプレート数(調整済み):matchT_prev="<<matchT_prev<< std::endl;
        for (int i = 0; i < matchT_prev; i++) {
          std::cout <<"MT_prev_pixel["<<i<<"]="<<MT_prev_pixel[i]<< std::endl;
        }*/

        //cv::line(img_tate,cv::Point(MT_curr2_pixel[0].x,MT_curr2_pixel[0].y),cv::Point(MT_prev_pixel[0].x,MT_prev_pixel[0].y),cv::Scalar(0,0,0), 2, cv::LINE_AA);
        //cv::line(img_tate,cv::Point(MT_curr2_pixel[1].x,MT_curr2_pixel[1].y),cv::Point(MT_prev_pixel[1].x,MT_prev_pixel[1].y),cv::Scalar(0,0,255), 2, cv::LINE_AA);

        //世界座標推定(マーカー観測不能時)------------------------------------------------------
        //マーカーが観測できない場合(観測可能な場合は2回目動作で世界座標を推定する)
        //新規テンプレートに一番近いキープテンプレートの世界座標を使用する(カメラ観測座標で)
        MT_curr_world.resize(1000);//初期設定
        if(markerIds.size() <= 0){
          std::cout <<"マーカー観測不能時"<< std::endl;
          double minlengh=1000000;
          int minMC;
          //std::cout <<"DMT_curr_ok="<<DMT_curr_ok<< std::endl;
          //std::cout <<"DMT_curr2_ok="<<DMT_curr2_ok<< std::endl;
          if(DMT_curr2_ok!=0){
            for (int i = 0; i < DMT_curr_ok; i++) {
              for(int j = 0; j < DMT_curr2_ok; j++){
                length=sqrt((MT_curr_camera[i].x-MT_curr2_camera[j].x)*(MT_curr_camera[i].x-MT_curr2_camera[j].x)
                +(MT_curr_camera[i].y-MT_curr2_camera[j].y)*(MT_curr_camera[i].y-MT_curr2_camera[j].y)
                +(MT_curr_camera[i].z-MT_curr2_camera[j].z)*(MT_curr_camera[i].z-MT_curr2_camera[j].z));
                if(minlengh>length){
                  minMC=j;
                  minlengh=length;
                }
              }
              std::cout <<"minMC["<<i<<"]="<<minMC<<std::endl;

              MT_curr_world[i].x=MT_curr2_world[minMC].x+(MT_curr_camera[i].x-MT_curr2_camera[minMC].x)*cos(-Est_RobotTH)-(MT_curr_camera[i].z-MT_curr2_camera[minMC].z)*sin(-Est_RobotTH);
              MT_curr_world[i].y=MT_curr2_world[minMC].y+(MT_curr_camera[i].y-MT_curr2_camera[minMC].y);
              MT_curr_world[i].z=MT_curr2_world[minMC].z+(MT_curr_camera[i].x-MT_curr2_camera[minMC].x)*sin(-Est_RobotTH)+(MT_curr_camera[i].z-MT_curr2_camera[minMC].z)*cos(-Est_RobotTH);

              //std::cout <<"MT_curr2_camera["<<minMC<<"]= "<<MT_curr2_camera[minMC]<< std::endl;
              //std::cout <<"MT_curr_camera["<<i<<"]= "<<MT_curr_camera[i]<< std::endl;
              //std::cout <<"MT_curr2_world["<<minMC<<"]= "<<MT_curr2_world[minMC]<< std::endl;
              //std::cout <<"MT_curr_world["<<i<<"]=  "<<MT_curr_world[i]<< std::endl;

              Test_MT_C2_Cy<<MT_curr2_camera[minMC].x<<"\n";
              Test_MT_C2_Cx<<MT_curr2_camera[minMC].z<<"\n";
              Test_MT_C1_Cy<<MT_curr_camera[i].x<<"\n";
              Test_MT_C1_Cx<<MT_curr_camera[i].z<<"\n";
              Test_MT_C2_Wy<<MT_curr2_world[minMC].x<<"\n";
              Test_MT_C2_Wx<<MT_curr2_world[minMC].z<<"\n";
              Test_MT_C1_Wy<<MT_curr_world[i].x<<"\n";
              Test_MT_C1_Wx<<MT_curr_world[i].z<<"\n";
              Test_MT_kaisu<<ALLrealsec<<"\n";

              MP_kaisu<<ALLrealsec<<"\n";
              MP_worldy<<MT_curr_world[i].x<<"\n";
              MP_worldx<<MT_curr_world[i].z<<"\n";
              MC_worldy<<"\n";
              MC_worldx<<"\n";
              MP_cameray<<MT_curr_camera[i].x<<"\n";
              MP_camerax<<MT_curr_camera[i].z<<"\n";
              MC_cameray<<"\n";
              MC_camerax<<"\n";
            }
            Test0_MT_C2_Cy<<MT_curr2_camera[minMC].x<<"\n";
            Test0_MT_C2_Cx<<MT_curr2_camera[minMC].z<<"\n";
            Test0_MT_C1_Cy<<MT_curr_camera[0].x<<"\n";
            Test0_MT_C1_Cx<<MT_curr_camera[0].z<<"\n";
            Test0_MT_C2_Wy<<MT_curr2_world[minMC].x<<"\n";
            Test0_MT_C2_Wx<<MT_curr2_world[minMC].z<<"\n";
            Test0_MT_C1_Wy<<MT_curr_world[0].x<<"\n";
            Test0_MT_C1_Wx<<MT_curr_world[0].z<<"\n";
            Test0_MT_kaisu<<ALLrealsec<<"\n";
          }
           //一つ前のテンプレートが追跡不能であった場合はマッチング前のテンプレートデータを利用する
          else if(DMT_curr2_ok==0){
            for (int i = 0; i < DMT_curr_ok; i++) {
              for(int j = 0; j < EST_MT_ok; j++){
                length=sqrt((MT_curr_camera[i].x-MT_prev_camera[j].x)*(MT_curr_camera[i].x-MT_prev_camera[j].x)
                +(MT_curr_camera[i].y-MT_prev_camera[j].y)*(MT_curr_camera[i].y-MT_prev_camera[j].y)
                +(MT_curr_camera[i].z-MT_prev_camera[j].z)*(MT_curr_camera[i].z-MT_prev_camera[j].z));
                if(minlengh>length){
                  minMC=j;
                  minlengh=length;
                }
              }
              std::cout <<"minMC["<<i<<"]="<<minMC<<std::endl;

              MT_curr_world[i].x=MT_prev_world[minMC].x+(MT_curr_camera[i].x-MT_prev_camera[minMC].x)*cos(-Est_RobotTH)-(MT_curr_camera[i].z-MT_prev_camera[minMC].z)*sin(-Est_RobotTH);
              MT_curr_world[i].y=MT_prev_world[minMC].y+(MT_curr_camera[i].y-MT_prev_camera[minMC].y);
              MT_curr_world[i].z=MT_prev_world[minMC].z+(MT_curr_camera[i].x-MT_prev_camera[minMC].x)*sin(-Est_RobotTH)+(MT_curr_camera[i].z-MT_prev_camera[minMC].z)*cos(-Est_RobotTH);
              
              //std::cout <<"MT_prev_camera["<<minMC<<"]= "<<MT_prev_camera[minMC]<< std::endl;
              //std::cout <<"MT_curr_camera["<<i<<"]= "<<MT_curr_camera[i]<< std::endl;
              //std::cout <<"MT_prev_world["<<minMC<<"]= "<<MT_prev_world[minMC]<< std::endl;
              //std::cout <<"MT_curr_world["<<i<<"]=  "<<MT_curr_world[i]<< std::endl;

              Test_MT_C2_Cy0<<MT_prev_camera[minMC].x<<"\n";
              Test_MT_C2_Cx0<<MT_prev_camera[minMC].z<<"\n";
              Test_MT_C1_Cy0<<MT_curr_camera[i].x<<"\n";
              Test_MT_C1_Cx0<<MT_curr_camera[i].z<<"\n";
              Test_MT_C2_Wy0<<MT_prev_world[minMC].x<<"\n";
              Test_MT_C2_Wx0<<MT_prev_world[minMC].z<<"\n";
              Test_MT_C1_Wy0<<MT_curr_world[i].x<<"\n";
              Test_MT_C1_Wx0<<MT_curr_world[i].z<<"\n";
              Test_MT_kaisu0<<ALLrealsec<<"\n";
            }
          }
          MT_curr_world.resize(DMT_curr_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
        }//if(markerIds.size() <= 0)→END

        //新規テンプレート追加動作-------------------------------------------------------
        //同じ所に作成されたテンプレート削除する(画像座標を使って比較)
        //3回目のマッチング結果(Prev)と2回目に作成した新規テンプレート(curr)を比較する
        //新規テンプレートと旧テンプレートを比較して全ての旧テンプレートと距離がテンプレートサイズ以上離れていたら追加する
        //距離がテンプレートサイズ以内ならばPrev(旧テンプレート)を削除し新規テンプレートを追加する
        //複数重なる場合は最後
        for (int i = 0; i < DMT_curr2_ok; i++) {
          double remave=0,minlengh=1000000;
          int minj;
          //std::cout <<"\n"<< std::endl;
          for (int j = 0; j < DMT_curr_ok; j++) {
            length=sqrt((MT_curr2_pixel[i].x-MT_curr_pixel[j].x)*(MT_curr2_pixel[i].x-MT_curr_pixel[j].x)
                        +(MT_curr2_pixel[i].y-MT_curr_pixel[j].y)*(MT_curr2_pixel[i].y-MT_curr_pixel[j].y));

            //std::cout <<"length="<<length<< std::endl;
            //std::cout <<"template_size="<<template_size<< std::endl;
            //std::cout <<"更新前:MT_curr_pixel["<<j<<"]="<<MT_curr_pixel[j]<< std::endl;
            //std::cout <<"更新前:MT_curr2_pixel["<<i<<"]="<<MT_curr2_pixel[i]<< std::endl;
            
            //更新動作(テンプレートのかぶりがあるとき→最も距離が近いテンプレートで更新する)
            if(length<template_size*2){
              remave=1;
              if(minlengh>length){
                minj=j;//最も距離が近いテンプレートの配列を保存
                minlengh=length;//最小距離の更新
              }
            }
          }//for (int j = 0; j < DMT_curr_ok; j++)→end
          //更新動作(テンプレートのかぶりが存在する時)
          if(remave==1){
            //std::cout <<"更新動作------------------------------------------------------------------------"<<std::endl;
            MT_curr2_Templ[i] = MT_curr_Templ[minj];//Depth取得可能なマッチテンプレート
            MT_curr2_pixel[i] = MT_curr_pixel[minj];//Depth取得可能なマッチング中心画像座標
            MT_curr2_camera[i] = MT_curr_camera[minj];
            MT_curr2_world[i] = MT_curr_world[minj];
            //更新したCurrを削除する(重複防止)
            int j,k;
            for ( j = k = 0; j < DMT_curr_ok; j++) {
              if(j!=minj){
                MT_curr_Templ[k]=MT_curr_Templ[j];
                MT_curr_pixel[k]=MT_curr_pixel[j];
                MT_curr_camera[k]=MT_curr_camera[j];
                MT_curr_world[k++]=MT_curr_world[j];
              }
            }
            DMT_curr_ok=k;
          }
        }//for (int i = 0; i < DMT_curr2_ok; i++) →end
        //追加動作(残ったCurrをCurr2に追加する→残ったcurrには更新要素が無いためテンプレートがかぶってない)
        for (int j = 0; j < DMT_curr_ok; j++) {
          MT_curr2_Templ[DMT_curr2_ok] = MT_curr_Templ[j];//Depth取得可能なマッチテンプレート
          MT_curr2_pixel[DMT_curr2_ok] = MT_curr_pixel[j];//Depth取得可能なマッチング中心画像座標
          MT_curr2_camera[DMT_curr2_ok] = MT_curr_camera[j];
          MT_curr2_world[DMT_curr2_ok] = MT_curr_world[j];
          //std::cout <<"追加動作:MT_curr2_pixel["<<DMT_curr2_ok<<"]="<<MT_curr2_pixel[DMT_curr2_ok]<< std::endl;
          //std::cout <<"追加動作:MT_curr2_camera["<<DMT_curr2_ok<<"]="<<MT_curr2_camera[DMT_curr2_ok]<< std::endl;
          DMT_curr2_ok=DMT_curr2_ok+1;//Depthが取得可能な全マッチテンプレート数
        }

        MT_curr2_pixel.resize(DMT_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
        MT_curr2_camera.resize(DMT_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)
        MT_curr2_world.resize(DMT_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
        //std::cout <<"全テンプレート数:DMT_prev_ok="<<DMT_curr2_ok<< std::endl;

        for (int i = 0; i < DMT_curr2_ok; i++) {
          //std::cout <<"MT_curr2_pixel["<<i<<"]="<<MT_curr2_pixel[i]<< std::endl;
          //テンプレートまでの距離と角度を求める(観測値)
          CameraLMT[0]=sqrt((MT_curr2_camera[i].x*MT_curr2_camera[i].x)+(MT_curr2_camera[i].z*MT_curr2_camera[i].z));
          CameraLMT[1]=atan2(MT_curr2_camera[i].x,MT_curr2_camera[i].z);
          Zu_T[i] = (cv::Mat_<double>(2,1) <<
            CameraLMT[0],
            CameraLMT[1]);
        }
      }//if(Tracking == true)→end
    }//if (reset == false)→end

    //マーカーTF-------------------------------------------------------------------------------------------
    /*std::string target_maker_frame = "marker_world_link";//cameraとマーカー間のリンク
    geometry_msgs::Pose maker_pose;

    maker_pose.position.x = MarkerW[markerIds.at(0)](2,0);//Rvizと画像は座標系が異なるので注意
    maker_pose.position.y = MarkerW[markerIds.at(0)](1,0);
    maker_pose.position.z = MarkerW[markerIds.at(0)](0,0);
    maker_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_maker;
    tf::Transform maker_transform;
    poseMsgToTF(maker_pose, maker_transform);
    br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), source_frame, target_maker_frame));*/

    if(Tracking == true){
      //tf(観測特徴点)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame1 = "MT_curr2_world";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose1;

      //std::cout << "tf特徴点の世界座標:MT_curr2_world[i]={x="<< MT_curr2_world[0].x <<",y="<<MT_curr2_world[0].y<<",z="<<MT_curr2_world[0].z<<"}"<< std::endl;
      maker_pose1.position.x = MT_curr2_world[0].z;//Rvizと画像は座標系が異なるので注意
      maker_pose1.position.y = MT_curr2_world[0].x;
      maker_pose1.position.z = MT_curr2_world[0].y;
      maker_pose1.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker1;
      tf::Transform maker_transform1;
      poseMsgToTF(maker_pose1, maker_transform1);
      br_maker1.sendTransform(tf::StampedTransform(maker_transform1, ros::Time::now(), source_frame, target_maker_frame1));
    }

    //ロボットの状態方程式
    //雑音の影響を考慮した実際の状態(カルマン推定前)
    if(robot_odometry.twist.twist.angular.z==0){
      Act_RobotX=Act_RobotX+(Act_RobotV*cos(Act_RobotTH)*realsec);
      Act_RobotY=Act_RobotY+(Act_RobotV*sin(Act_RobotTH)*realsec);
      Act_RobotTH=Act_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
    }
    else{
      Act_RobotX=Act_RobotX+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(sin(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Act_RobotTH)));
      Act_RobotY=Act_RobotY+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(-cos(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)+cos(Act_RobotTH)));
      Act_RobotTH=Act_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
    }

    //std::cout << "Act_RobotX=" <<Act_RobotX<< std::endl;
    //std::cout << "Act_RobotY=" <<Act_RobotY<< std::endl;
    //std::cout << "Act_RobotTH=" <<Act_RobotTH<< std::endl;

    ACT_Robot= (cv::Mat_<double>(3,1) <<
      Act_RobotX,
      Act_RobotY,
      Act_RobotTH);

    if(Act_RobotX>0||Act_RobotY>0||Act_RobotTH>0){//ラグの調整
      ////目標指令状態
      if(robot_velocity.angular.z==0){
        Des_RobotX=Des_RobotX+(Des_RobotV*cos(Des_RobotTH)*realsec);
        Des_RobotY=Des_RobotY+(Des_RobotV*sin(Des_RobotTH)*realsec);
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
      else{
        Des_RobotX=Des_RobotX+((Des_RobotV/robot_velocity.angular.z)*(sin(Des_RobotTH+robot_velocity.angular.z*realsec)-sin(Des_RobotTH)));
        Des_RobotY=Des_RobotY+((Des_RobotV/robot_velocity.angular.z)*(-cos(Des_RobotTH+robot_velocity.angular.z*realsec)+cos(Des_RobotTH)));
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
    }

    //std::cout << "Des_RobotX=" <<Des_RobotX<< std::endl;
    //std::cout << "Des_RobotY=" <<Des_RobotY<< std::endl;
    //std::cout << "Des_RobotTH=" <<Des_RobotTH<< std::endl;

    DES_Robot= (cv::Mat_<double>(3,1) <<
      Des_RobotX,
      Des_RobotY,
      Des_RobotTH);

    if(Act_RobotX>0||Act_RobotY>0||Act_RobotTH>0){//ラグの調整
      //予測ステップ-----------------------------------------------------------------------------------------------------
      std::cout <<"予測ステップ"<< std::endl;
      //Mt誤差共分散(指令値に発生するノイズ)
      //指令値に事前分散をかけたもの
      Mt= (cv::Mat_<double>(2,2) << 
          ((2.97694E-5*abs(Act_RobotV))+(0.000126947*abs(robot_odometry.twist.twist.angular.z)))/realsec, 0,
          0, ((3.30119E-6*abs(Act_RobotV))+(0.00110641*abs(robot_odometry.twist.twist.angular.z)))/realsec);

      //事前誤差共分散を距離1[mm]角度1[°]=0.017[rad]に変更
      //Mt= (cv::Mat_<double>(2,2) << 
      //    ((1.000E-3*abs(Des_RobotV))+(0.000126947*abs(robot_odometry.twist.twist.angular.z)))/realsec, 0,
      //    0, ((3.30119E-6*abs(Des_RobotV))+(0.01745329*abs(robot_odometry.twist.twist.angular.z)))/realsec);
      double At00,At01,At10,At11,At20,At21;

      //Atは速度、角速度は指令値、角度θ(状態)は一つ前の実測状態を使用
      if(robot_odometry.twist.twist.angular.z==0){
        At00=cos(Act_RobotTH)*realsec;
        At01=0;
        At10=sin(Act_RobotTH)*realsec;
        At11=0;
        At20=0;
        At21=realsec;
      }
      else{
        At00=(1/robot_odometry.twist.twist.angular.z)*(sin(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Act_RobotTH));
        At01=-(Act_RobotV/(robot_odometry.twist.twist.angular.z*robot_odometry.twist.twist.angular.z))*(sin(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Act_RobotTH))
            +((Act_RobotV/robot_odometry.twist.twist.angular.z)*realsec*cos(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec));
        At10=(1/robot_odometry.twist.twist.angular.z)*(-cos(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)+cos(Act_RobotTH));
        At11=-(Act_RobotV/(robot_odometry.twist.twist.angular.z*robot_odometry.twist.twist.angular.z))*(-cos(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)+cos(Act_RobotTH))
            +((Act_RobotV/robot_odometry.twist.twist.angular.z)*realsec*sin(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec));
        At20=0;
        At21=realsec;
      }

      //実際の出力と指令値との誤差を入れるためのヤコビアン
      At= (cv::Mat_<double>(3,2) <<
          At00, At01,
          At10, At11,
          At20, At21);

      //Ftは速度、角速度は指令値、角度θは一つ前の推定状態を使用
      if(robot_odometry.twist.twist.angular.z==0){
        Ft= (cv::Mat_<double>(3,3) << 
          1, 0, -Act_RobotV*sin(Est_RobotTH)*realsec,
          0, 1, Act_RobotV*cos(Est_RobotTH)*realsec,
          0, 0, 1);
      }
      else{
        Ft= (cv::Mat_<double>(3,3) << 
          1, 0, (Act_RobotV/robot_odometry.twist.twist.angular.z)*(cos(Est_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-cos(Est_RobotTH)),
          0, 1, (Act_RobotV/robot_odometry.twist.twist.angular.z)*(sin(Est_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Est_RobotTH)),
          0, 0, 1);
      }
      //信念分布の共分散行列式Σ(3☓3)(発生する誤差はここにまとまっている)
      //std::cout <<"Cov="<<Cov<< std::endl;
      //std::cout <<"Ft="<<Ft<< std::endl;
      //std::cout <<"At="<<At<< std::endl;
      //std::cout <<"Mt="<<Mt<< std::endl;

      Cov=Ft*Cov*Ft.t()+At*Mt*At.t();
      //std::cout <<"Cov="<<Cov<< std::endl;
      kal_covX1<<Cov.at<double>(0,0)<<"\n";
      kal_covX2<<Cov.at<double>(0,1)<<"\n";
      kal_covX3<<Cov.at<double>(0,2)<<"\n";
      kal_covY1<<Cov.at<double>(1,0)<<"\n";
      kal_covY2<<Cov.at<double>(1,1)<<"\n";
      kal_covY3<<Cov.at<double>(1,2)<<"\n";
      kal_covTH1<<Cov.at<double>(2,0)<<"\n";
      kal_covTH2<<Cov.at<double>(2,1)<<"\n";
      kal_covTH3<<Cov.at<double>(2,2)<<"\n";
      kal_cov_time<<ALLrealsec<<"\n";

      //信念分布の中心(平均)μ(一つ前の推定位置(信念分布の中心)と指令値)
      if(robot_odometry.twist.twist.angular.z==0){
        Est_RobotX=Est_RobotX+(Act_RobotV*cos(Est_RobotTH)*realsec);
        Est_RobotY=Est_RobotY+(Act_RobotV*sin(Est_RobotTH)*realsec);
        Est_RobotTH=Est_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
      }
      else{
        Est_RobotX=Est_RobotX+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(sin(Est_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Est_RobotTH)));
        Est_RobotY=Est_RobotY+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(-cos(Est_RobotTH+robot_odometry.twist.twist.angular.z*realsec)+cos(Est_RobotTH)));
        Est_RobotTH=Est_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
      }
      std::cout << "Est_RobotX=" <<Est_RobotX<< std::endl;
      std::cout << "Est_RobotY=" <<Est_RobotY<< std::endl;
      std::cout << "Est_RobotTH=" <<Est_RobotTH<< std::endl;

      EST_Robot= (cv::Mat_<double>(3,1) <<
        Est_RobotX,
        Est_RobotY,
        Est_RobotTH);

      //更新ステップ(マーカー)------------------------------------------------------------------
      //更新ステップはランドーマークの数だけ更新を行う
      /*for(int i=0;i<markerIds.size();i++){
        std::cout <<"更新ステップ(マーカー)"<< std::endl;
        //観測方程式(信念分布の中心位置から見たLMまでの距離と角度)(理想推定)
        hu = (cv::Mat_<double>(2,1) <<
          sqrt((Est_RobotX-MarkerW[markerIds.at(i)](2,0))*(Est_RobotX-MarkerW[markerIds.at(i)](2,0)) + (Est_RobotY-MarkerW[markerIds.at(i)](0,0))*(Est_RobotY-MarkerW[markerIds.at(i)](0,0))),
          atan2(MarkerW[markerIds.at(i)](0,0)-Est_RobotY,MarkerW[markerIds.at(i)](2,0)-Est_RobotX) - Est_RobotTH);

        double lu=sqrt((Est_RobotX-MarkerW[markerIds.at(i)](2,0))*(Est_RobotX-MarkerW[markerIds.at(i)](2,0))+(Est_RobotY-MarkerW[markerIds.at(i)](0,0))*(Est_RobotY-MarkerW[markerIds.at(i)](0,0)));

        //観測方程式のヤコビ行列Ht信念分布の中心と世界座標ランドマーク位置(MarkerW[markerIds.at(i)](0,0),MarkerW[markerIds.at(i)](2,0))(理想推定)
        Ht= (cv::Mat_<double>(2,3) << //共分散Ht
          (Est_RobotX-MarkerW[markerIds.at(i)](2,0))/lu,       (Est_RobotY-MarkerW[markerIds.at(i)](0,0))/lu,       0,
          (MarkerW[markerIds.at(i)](0,0)-Est_RobotY)/(lu*lu),  (Est_RobotX-MarkerW[markerIds.at(i)](2,0))/(lu*lu), -1);

        Qt= (cv::Mat_<double>(2,2) <<//センサーの誤差共分散(直進と角度の誤差共分散を代入)
          (lu*lu)*0.0006,  0,
          0,            0.00008);
        
        //Qt= (cv::Mat_<double>(2,2) <<//センサーの誤差共分散(直進と角度の誤差共分散を代入)
        //  (lu*lu)*1.83216E-5,  0,
        //  0,            0.000000008);

        cv::Mat tempK=Qt+(Ht*Cov*Ht.t());
        K=Cov*Ht.t()*tempK.inv();//カルマンゲインK

        kal_gainX1<<K.at<double>(0,0)<<"\n";
        kal_gainX2<<K.at<double>(0,1)<<"\n";
        kal_gainY1<<K.at<double>(1,0)<<"\n";
        kal_gainY2<<K.at<double>(1,1)<<"\n";
        kal_gainTH1<<K.at<double>(2,0)<<"\n";
        kal_gainTH2<<K.at<double>(2,1)<<"\n";
        kal_gain_time<<ALLrealsec<<"\n";

        //センサー値を反映するための更新式
        Cov=(I-K*Ht)*Cov;//信念分布の分散Σ
        kal_covX1<<Cov.at<double>(0,0)<<"\n";
        kal_covX2<<Cov.at<double>(0,1)<<"\n";
        kal_covX3<<Cov.at<double>(0,2)<<"\n";
        kal_covY1<<Cov.at<double>(1,0)<<"\n";
        kal_covY2<<Cov.at<double>(1,1)<<"\n";
        kal_covY3<<Cov.at<double>(1,2)<<"\n";
        kal_covTH1<<Cov.at<double>(2,0)<<"\n";
        kal_covTH2<<Cov.at<double>(2,1)<<"\n";
        kal_covTH3<<Cov.at<double>(2,2)<<"\n";
        kal_cov_time<<ALLrealsec<<"\n";

        //信念分布の中心位置(推定自己位置)μ=EST_Robot
        //zはセンサーから取得したマーカーまでの距離と角度データ
        //hは推定位置からみたマーカーまでの距離と角度(理想推定値)
        //多分カルマンゲインをかけることで2次から3次への拡張が可能
        EST_Robot=K*(Zu[markerIds.at(i)]-hu)+EST_Robot;
        kansoku_Zu1<<Zu[markerIds.at(i)].at<double>(0,0)<<"\n";
        kansoku_Zu2<<Zu[markerIds.at(i)].at<double>(1,0)<<"\n";
        kansoku_hu1<<hu.at<double>(0,0)<<"\n";
        kansoku_hu2<<hu.at<double>(1,0)<<"\n";
      }*/

      //更新ステップ(特徴線)------------------------------------------------------------------
      //更新ステップは特徴線の数だけ更新を行う
      for(int i=0;i<DMT_curr2_ok;i++){
        //std::cout <<"更新ステップ(特徴線)"<< std::endl;
        //観測方程式(信念分布の中心位置から見たLMまでの距離と角度)(理想推定)
        hu = (cv::Mat_<double>(2,1) <<
          sqrt((Est_RobotX-MT_curr2_world[i].z)*(Est_RobotX-MT_curr2_world[i].z) + (Est_RobotY-MT_curr2_world[i].x)*(Est_RobotY-MT_curr2_world[i].x)),
          atan2(MT_curr2_world[i].x-Est_RobotY,MT_curr2_world[i].z-Est_RobotX) - Est_RobotTH);
        double lu=sqrt((Est_RobotX-MT_curr2_world[i].z)*(Est_RobotX-MT_curr2_world[i].z)+(Est_RobotY-MT_curr2_world[i].x)*(Est_RobotY-MT_curr2_world[i].x));

        //観測方程式のヤコビ行列Ht信念分布の中心と世界座標ランドマーク位置(MT_curr2_world[i].x,MT_curr2_world[i].z)(理想推定)
        Ht= (cv::Mat_<double>(2,3) << //共分散Ht
          (Est_RobotX-MT_curr2_world[i].z)/lu,       (Est_RobotY-MT_curr2_world[i].x)/lu,       0,
          (MT_curr2_world[i].x-Est_RobotY)/(lu*lu),  (Est_RobotX-MT_curr2_world[i].z)/(lu*lu), -1);

        Qt= (cv::Mat_<double>(2,2) <<//センサーの誤差共分散(直進と角度の誤差共分散を代入)
          (lu*lu)*0.0006,  0,
          0,            0.00008);

        //Qt= (cv::Mat_<double>(2,2) <<//センサーの誤差共分散(直進と角度の誤差共分散を代入)
        //  (lu*lu)*1.83216E-5,  0,
        //  0,            0.000000008);

        //カルマンゲインK
        cv::Mat tempK=Qt+(Ht*Cov*Ht.t());
        K=Cov*Ht.t()*tempK.inv();
        kal_gainX1<<K.at<double>(0,0)<<"\n";
        kal_gainX2<<K.at<double>(0,1)<<"\n";
        kal_gainY1<<K.at<double>(1,0)<<"\n";
        kal_gainY2<<K.at<double>(1,1)<<"\n";
        kal_gainTH1<<K.at<double>(2,0)<<"\n";
        kal_gainTH2<<K.at<double>(2,1)<<"\n";
        kal_gain_time<<ALLrealsec<<"\n";

        //センサー値を反映するための更新式
        Cov=(I-K*Ht)*Cov;//信念分布の分散Σ
        kal_covX1<<Cov.at<double>(0,0)<<"\n";
        kal_covX2<<Cov.at<double>(0,1)<<"\n";
        kal_covX3<<Cov.at<double>(0,2)<<"\n";
        kal_covY1<<Cov.at<double>(1,0)<<"\n";
        kal_covY2<<Cov.at<double>(1,1)<<"\n";
        kal_covY3<<Cov.at<double>(1,2)<<"\n";
        kal_covTH1<<Cov.at<double>(2,0)<<"\n";
        kal_covTH2<<Cov.at<double>(2,1)<<"\n";
        kal_covTH3<<Cov.at<double>(2,2)<<"\n";
        kal_cov_time<<ALLrealsec<<"\n";

        //信念分布の中心位置(推定自己位置)μ=EST_Robot
        //zはセンサーから取得したマーカーまでの距離と角度データ
        //hは推定位置からみたマーカーまでの距離と角度(理想推定値)
        //多分カルマンゲインをかけることで2次から3次への拡張が可能
        EST_Robot=K*(Zu_T[i]-hu)+EST_Robot;
        //std::cout <<"K="<<K<< std::endl;
        //std::cout <<"Zu_T[i]="<<Zu_T[i]<< std::endl;
        //std::cout <<"hu="<<hu<< std::endl;

        kansoku_Zu1<<Zu_T[i].at<double>(0,0)<<"\n";
        kansoku_Zu2<<Zu_T[i].at<double>(1,0)<<"\n";
        kansoku_hu1<<hu.at<double>(0,0)<<"\n";
        kansoku_hu2<<hu.at<double>(1,0)<<"\n";
      }


      Est_RobotX=EST_Robot.at<double>(0);
      Est_RobotY=EST_Robot.at<double>(1);
      Est_RobotTH=EST_Robot.at<double>(2);

      //std::cout <<"Act_RobotX="<<Act_RobotX<< std::endl;
      //std::cout <<"Act_RobotY="<<Act_RobotY<< std::endl;
      //std::cout <<"Act_RobotTH="<<Act_RobotTH<< std::endl;
      //std::cout <<"Est_RobotX="<<Est_RobotX<< std::endl;
      //std::cout <<"Est_RobotY="<<Est_RobotY<< std::endl;
      //std::cout <<"Est_RobotTH="<<Est_RobotTH<< std::endl;

      est_kaisu<<ALLrealsec<<"\n";//カルマン更新動作確認用
      est_robotxx<<Est_RobotX<<"\n";
      est_robotyy<<Est_RobotY<<"\n";
      est_robotthth<<Est_RobotTH<<"\n";

      act_robotxx<<Act_RobotX<<"\n";
      act_robotyy<<Act_RobotY<<"\n";
      act_robotthth<<Act_RobotTH<<"\n";
    }
  }//if(time0 != false)→END
  
  std::cout<<"----------------------前方障害物まで残り"<<img_depth.at<float>(239,320)/1000<<"[m]-----------------------------------------"<<std::endl;

  //Rviz関連
  if(kaisu>1){
    //指令値状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
    //ここがカメラの姿勢部分
    std::string MaptoDes_Robot_frame = "MaptoDes_Robot_link";
    double RollX=0,PitchY=Des_RobotTH,YawZ=0;//微小区間回転行列

    //カメラ位置(理論値)
    Des_Robot_pose.position.x = Des_RobotX;//赤
    Des_Robot_pose.position.y = Des_RobotY;//緑
    Des_Robot_pose.position.z = 0;//青(tf:Z,画像:Y)
    Des_Robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

    static tf::TransformBroadcaster br_Des_Robot_pose;
    tf::Transform Des_Robot_transform;
    poseMsgToTF(Des_Robot_pose, Des_Robot_transform);
    br_Des_Robot_pose.sendTransform(tf::StampedTransform(Des_Robot_transform, ros::Time::now(), source_frame, MaptoDes_Robot_frame));
    //std::cout <<"Des_Robot_pose.position.x="<<Des_Robot_pose.position.x<< std::endl;
    //std::cout <<"Des_Robot_pose.position.y="<<Des_Robot_pose.position.y<< std::endl;
    //std::cout <<"Des_Robot_pose.orientation.z="<<Des_Robot_pose.orientation.z<< std::endl;
    //std::cout <<"Des_Robot_pose.orientation.w="<<Des_Robot_pose.orientation.w<< std::endl;

    //tf(Des_Robot camera_link間のlink)-----------------------------------------------------------------------------------------
    geometry_msgs::Pose Des_Robot_Link_pose;
    std::string Des_Robot_camera_frame = "Des_Robot_camera_link";
    Des_Robot_Link_pose.position.x = 0;
    Des_Robot_Link_pose.position.y = 0;
    Des_Robot_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
    Des_Robot_Link_pose.orientation.w = 1.0;
    static tf::TransformBroadcaster br_Des_Robot_Link_pose;
    tf::Transform Des_Robot_Link_transform;
    poseMsgToTF(Des_Robot_Link_pose, Des_Robot_Link_transform);
    br_Des_Robot_Link_pose.sendTransform(tf::StampedTransform(Des_Robot_Link_transform, ros::Time::now(), MaptoDes_Robot_frame, Des_Robot_camera_frame));
    
    //観測状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
    //ここがカメラの姿勢部分
    std::string MaptoAct_Robot_frame = "MaptoAct_Robot_link";
    RollX=0,PitchY=Act_RobotTH,YawZ=0;//微小区間回転行列

    //カメラ位置(理論値)
    Act_Robot_pose.position.x = Act_RobotX;//赤
    Act_Robot_pose.position.y = Act_RobotY;//緑
    Act_Robot_pose.position.z = 0;//青(tf:Z,画像:Y)
    Act_Robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

    static tf::TransformBroadcaster br_Act_Robot_pose;
    tf::Transform Act_Robot_transform;
    poseMsgToTF(Act_Robot_pose, Act_Robot_transform);
    br_Act_Robot_pose.sendTransform(tf::StampedTransform(Act_Robot_transform, ros::Time::now(), source_frame, MaptoAct_Robot_frame));
    //std::cout <<"Act_Robot_pose.position.x="<<Act_Robot_pose.position.x<< std::endl;
    //std::cout <<"Act_Robot_pose.position.y="<<Act_Robot_pose.position.y<< std::endl;
    //std::cout <<"Act_Robot_pose.orientation.z="<<Act_Robot_pose.orientation.z<< std::endl;
    //std::cout <<"Act_Robot_pose.orientation.w="<<Act_Robot_pose.orientation.w<< std::endl;

    //tf(Act_Robot camera_link間のlink)-----------------------------------------------------------------------------------------
    geometry_msgs::Pose Act_Robot_Link_pose;
    std::string Act_Robot_camera_frame = "Act_Robot_camera_link";

    //std::string Act_Robot_Link_Link_frame = "Act_Robot_Link_Link_link";
    Act_Robot_Link_pose.position.x = 0;
    Act_Robot_Link_pose.position.y = 0;
    Act_Robot_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
    Act_Robot_Link_pose.orientation.w = 1.0;
    static tf::TransformBroadcaster br_Act_Robot_Link_pose;
    tf::Transform Act_Robot_Link_transform;
    poseMsgToTF(Act_Robot_Link_pose, Act_Robot_Link_transform);
    br_Act_Robot_Link_pose.sendTransform(tf::StampedTransform(Act_Robot_Link_transform, ros::Time::now(), MaptoAct_Robot_frame, Act_Robot_camera_frame));

    //推定状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
    //ここがカメラの姿勢部分
    std::string MaptoEst_Robot_frame = "MaptoEst_Robot_link";
    RollX=0,PitchY=Est_RobotTH,YawZ=0;//微小区間回転行列

    //カメラ位置(理論値)
    Est_Robot_pose.position.x = Est_RobotX;//赤
    Est_Robot_pose.position.y = Est_RobotY;//緑
    Est_Robot_pose.position.z = 0;//青(tf:Z,画像:Y)
    Est_Robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

    static tf::TransformBroadcaster br_Est_Robot_pose;
    tf::Transform Est_Robot_transform;
    poseMsgToTF(Est_Robot_pose, Est_Robot_transform);
    br_Est_Robot_pose.sendTransform(tf::StampedTransform(Est_Robot_transform, ros::Time::now(), source_frame, MaptoEst_Robot_frame));
    //std::cout <<"Est_Robot_pose.position.x="<<Est_Robot_pose.position.x<< std::endl;
    //std::cout <<"Est_Robot_pose.position.y="<<Est_Robot_pose.position.y<< std::endl;
    //std::cout <<"Est_Robot_pose.orientation.z="<<Est_Robot_pose.orientation.z<< std::endl;
    //std::cout <<"Est_Robot_pose.orientation.w="<<Est_Robot_pose.orientation.w<< std::endl;

    //tf(Est_Robot camera_link間のlink)-----------------------------------------------------------------------------------------
    geometry_msgs::Pose Est_Robot_Link_pose;
    std::string Est_Robot_camera_frame = "Est_Robot_camera_link";

    //std::string Est_Robot_Link_Link_frame = "Est_Robot_Link_Link_link";
    Est_Robot_Link_pose.position.x = 0;
    Est_Robot_Link_pose.position.y = 0;
    Est_Robot_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
    Est_Robot_Link_pose.orientation.w = 1.0;
    static tf::TransformBroadcaster br_Est_Robot_Link_pose;
    tf::Transform Est_Robot_Link_transform;
    poseMsgToTF(Est_Robot_Link_pose, Est_Robot_Link_transform);
    br_Est_Robot_Link_pose.sendTransform(tf::StampedTransform(Est_Robot_Link_transform, ros::Time::now(), MaptoEst_Robot_frame, Est_Robot_camera_frame));
  }

  else{
    Des_Robot_pose.position.x = 0;
    Des_Robot_pose.position.y = 0;
    Des_Robot_pose.position.z = 0;
    Des_Robot_pose.orientation.x=0;
    Des_Robot_pose.orientation.y=0;
    Des_Robot_pose.orientation.z=0;
    Des_Robot_pose.orientation.w=0;

    Act_Robot_pose.position.x = 0;
    Act_Robot_pose.position.y = 0;
    Act_Robot_pose.position.z = 0;
    Act_Robot_pose.orientation.x=0;
    Act_Robot_pose.orientation.y=0;
    Act_Robot_pose.orientation.z=0;
    Act_Robot_pose.orientation.w=0;

    Est_Robot_pose.position.x = 0;
    Est_Robot_pose.position.y = 0;
    Est_Robot_pose.position.z = 0;
    Est_Robot_pose.orientation.x=0;
    Est_Robot_pose.orientation.y=0;
    Est_Robot_pose.orientation.z=0;
    Est_Robot_pose.orientation.w=0;
  }

  //経路描写-------------------------------------------------------------
    Des_pose.header.stamp = ros::Time::now();
    Des_pose.header.frame_id = source_frame;
    Des_pose.pose.position = Des_Robot_pose.position;
    Des_pose.pose.orientation = Des_Robot_pose.orientation;
    Des_path.header.stamp = ros::Time::now();
    Des_path.header.frame_id = source_frame;
    Des_path.poses.push_back(Des_pose);
    Des_pub_plan.publish(Des_path);

    Act_pose.header.stamp = ros::Time::now();
    Act_pose.header.frame_id = source_frame;
    Act_pose.pose.position = Act_Robot_pose.position;
    Act_pose.pose.orientation = Act_Robot_pose.orientation;
    Act_path.header.stamp = ros::Time::now();
    Act_path.header.frame_id = source_frame;
    Act_path.poses.push_back(Act_pose);
    Act_pub_plan.publish(Act_path);

    Est_pose.header.stamp = ros::Time::now();
    Est_pose.header.frame_id = source_frame;
    Est_pose.pose.position = Est_Robot_pose.position;
    Est_pose.pose.orientation = Est_Robot_pose.orientation;
    Est_path.header.stamp = ros::Time::now();
    Est_path.header.frame_id = source_frame;
    Est_path.poses.push_back(Est_pose);
    Est_pub_plan.publish(Est_path);

   
    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_fld, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld_ty, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld_t, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld_y, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line3, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line4, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_graph, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_tate, cv::WINDOW_AUTOSIZE);

   
    cv::imshow(win_src, img_src);
    //cv::imshow(win_depth, img_depth);
    //cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_fld, img_fld);
    //cv::imshow(win_fld_ty, img_FLD_TY);
    //cv::imshow(win_fld_t, img_FLD_T);
    //cv::imshow(win_fld_y, img_FLD_Y);
    //cv::imshow(win_line2, img_line2);
    //cv::imshow(win_line3, img_line3);
    //cv::imshow(win_line4, img_line4);
    //cv::imshow(win_graph, img_graph);
    cv::imshow(win_tate, img_tate);//縦線関連の表示用

    //初回動作時＋検出時
    cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
    cv::swap(tate_point_curr, tate_point_prev);//縦線中点の画像座標を保存(tate_point_curr→tate_point_prev)
    cv::swap(TPCC, TPCP);//縦線中点の三次元カメラ座標を保存(TPCC→TPCP)
    cv::swap(TPCC_Templ, TPCP_Templ);//今のテンプレートを一つ前のテンプレートとして保存
    DTPP_ok=DTPC_ok;//テンプレート取得数を一つ前の取得数として保存()

    //2回目動作時
    if(Tracking==false){ 
      DMT_prev_ok=DMT_curr_ok;//Depth取得可能マッチテンプレート数をキープ+(3回目以降はこれに一つ前のテンプレート要素を追加する)
      cv::swap(MT_curr_world, MT_prev_world);//マッチ中心カメラ座標
      cv::swap(MT_curr_camera,MT_prev_camera);//マッチ中心カメラ座標
      cv::swap(MT_curr_pixel, MT_prev_pixel);//マッチ中心画像座標
      cv::swap(MT_curr_Templ, MT_prev_Templ);//マッチ座標
      Tracking=true;//3回目動作
    }
    //3回目以降動作時
    else if(Tracking == true){
      //Depth取得可能マッチング要素キープ
      DMT_prev_ok=DMT_curr2_ok;
      cv::swap(MT_curr2_world,  MT_prev_world);//マッチ中心カメラ座標
      cv::swap(MT_curr2_camera, MT_prev_camera);//マッチ中心カメラ座標
      cv::swap(MT_curr2_pixel,  MT_prev_pixel);//マッチ中心画像座標
      cv::swap(MT_curr2_Templ,  MT_prev_Templ);//マッチ座標
    }

    temp_kosuu<<DMT_prev_ok<<"\n";
    temp_time<<ALLrealsec<<"\n";  
    robot_velocity.linear.x  = 0.0; // 並進速度の初期化
    robot_velocity.angular.z = 0.0; // 回転速度の初期化
    kaisu++;
    time0=true;//一回目スキップ
    endTime=startTime;//動作終了時刻取得
    endTimeV1=startTimeV1;//動作終了時刻取得
    endTimeM1=startTimeM1;//動作終了時刻取得
	cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){
	ros::init(argc,argv,"FLD_clustering_3");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0) ros_begin = ros::Time::now();
  straight_begin = ros::Time::now();

  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nhSub, "/robot1/odom", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

    ros::NodeHandle nhPub;
    Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/Des_path",1000);
    Act_pub_plan = nhPub.advertise<nav_msgs::Path>("/Act_path",1000);
    Est_pub_plan = nhPub.advertise<nav_msgs::Path>("/Est_path",1000);

    ros::NodeHandle nh;
    pub= nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);

    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.5, 0.28, 3.0);//20211109廊下(直進2m,回転3.141592653/2.0,速度0.1,角速度(0.2+(0.0176*ALLrealsecM1+0.11)))
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<1.5, 0.28, 2.9);//実測値(X:3.42,Y:1.99)
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<4.0, 0.28, 2.0);

    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 3.0);//20211110廊下(直進40m,速度0.3)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 12.95);//実測値(X:3.42,Y:1.99)
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 27.67);

    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 3.0);//20211117_V1廊下(直進2m,速度0.3,回転90度,回転速度0.2,直進10m,速度0.3)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<9.52, 0.28, 0.77);//実測値(X:3.42,Y:1.99)

    //廊下回転動作実験
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 3.0);//20211117_V2廊下(直進2m,速度0.3,回転90度,回転速度0.2,直進10m,速度0.3)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<9.52, 0.28,2.9);//実測値(X:0.98,Y:6.23)
    //LX=2.0,VX=0.3,omegaZ=2,THZ=0.3;

    //廊下回転動作実験
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.165, 0.28, 3.0);//20211118_V1廊下(直進4.5m,速度0.3,回転90度,回転速度0.2,直進10m,速度0.3)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.61, 0.28, 5.8);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<7.57, 0.28, 3.0);
    

    //20211120V1廊下(直進4.5m,速度0.3,回転π/2度,回転速度0.2+α,直進8m,速度0.3)廊下回転動作実験
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.165, 0.28, 3.1);
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.61, 0.28, 4.8);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<5.867, 0.28, 4.8);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<7.57, 0.28, 3.1);
    //LX=4.5,VX=0.3,omegaZ=2,THZ=0.3;

    //20211120V2廊下(直進2.0m,速度0.25,回転90度,回転速度0.15+α,直進8m,速度0.25)廊下回転動作実験
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.55, 0.28, 2.85);//実測値(X:2.43,Y:4.81)ロボット座標系
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.55, 0.28, 2.85);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<8.53, 0.28, 1.08);
    //LX=2.0,VX=0.25,omegaZ=2,THZ=0.25;

  //20211123研究室(直進3.0m,速度0.25,回転-π/2.15度,回転速度0.15+α,直進2.0m,速度0.25)特徴が多い環境での自己位置推定
  //MarkerW[0]= (cv::Mat_<float>(3, 1) <<0.0, 0.28, 3.00);
  //MarkerW[1]= (cv::Mat_<float>(3, 1) <<-2.3, 0.28, 2.74);
  //LX=3.0,VX=0.25,omegaZ=2.15,THZ=0.25;

    //20211202V1廊下(直進2.0m,速度0.25,回転π/2度,回転速度0.15+α,直進5.0m,速度0.25)廊下回転動作実験
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.55, 0.28, 2.85);//実測値(X:2.22,4.25)
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.51, 0.28, 2.85);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<8.53, 0.28, 1.08);
    //LX=2.0,VX=0.25,omegaZ=2,THZ=0.25,LY=5.0;

    //20211202V2〜20211207V1廊下(直進2.0m,速度0.25,回転π/2度,回転速度0.15+α,直進5.0m,速度0.25)廊下回転動作実験
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.35, 0.28, 2.95);//実測値(X:2.22,4.25)
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.51, 0.28, 2.85);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<8.53, 0.28, 1.08);
    //LX=2.0,VX=0.25,omegaZ=2,THZ=0.25,LY=5.0;

    //20211207V1廊下(直進2.0m,速度0.25,回転π/2度,回転速度0.25,直進5.0m,速度0.25)廊下回転動作実験
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.35, 0.28, 2.95);//実測値(X:1.573,4.955)
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.51, 0.28, 2.85);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<8.53, 0.28, 1.08);
    //LX=2.0,VX=0.25,omegaZ=2,THZ=0.25,LY=5.0;

    //20211207V2廊下(直進12.0m,速度0.25)直線動作実験(研究室前スタート)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.905, 0.28, 3.0);//右手系(左方向X,縦方向Y,奥行きZ))
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<-0.932, 0.28, 7.754);
    //LX=12.0,VX=0.25,omegaZ=0,THZ=0,LY=0;

    //2022-01-08-V1廊下(直進2.0m,速度0.25,回転π/2度,回転速度0.20,直進5.0m,速度0.25)廊下回転動作実験
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.35, 0.28, 2.95);//実測値(X:1.57,Y:4.95)
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.51, 0.28, 2.85);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<8.53, 0.28, 1.08);
    //LX=2.0,VX=0.25,omegaZ=2.0,THZ=0.20,LY=5.0;

    //2022-01-08-V2廊下(直進2.0m,速度0.25,回転π/2度,回転速度0.20,直進5.0m,速度0.25)廊下回転動作実験
    //MarkerW[1]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 2);
    // MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.9, 0.28, 3);//実測値(X:1.57,Y:4.95)
    // MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.62, 0.28, 2.9);
    // MarkerW[6]= (cv::Mat_<float>(3, 1) <<2.62, 0.28, 0.77);
    // MarkerW[2]= (cv::Mat_<float>(3, 1) <<2.62, 0.28, 2.9);
    // MarkerW[1]= (cv::Mat_<float>(3, 1) <<0.9, 0.28, 3.0);
    // MarkerW[3]= (cv::Mat_<float>(3, 1) <<8.48, 0.28, 2.9);
    MarkerW[1]= (cv::Mat_<float>(3, 1) <<0, 0.28, 2.0);//(左、高さ、前)
    MarkerW[2]= (cv::Mat_<float>(3, 1) <<1.05, 0.28, 3.0);//(左、高さ、前)
    // MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.05, 0.28, 3.0);//(左、高さ、前)

    // LX=1.75,VX=0.25,omegaZ=2.05,THZ=0.10,LY=5.0;
    // VX = 0.25,THZ = 0.10;

    //2022-01-09ゼミ室前廊下(直進2.0m,速度0.25,回転π/2度,回転速度0.20,直進5.0m,速度0.25)廊下回転動作実験
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0, 0.28, 3.50);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.62, 0.28, 0.77);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<6.95, 0.28, 0.77);//差.4.33、20cm考慮4.13
    //MarkerW[1]= (cv::Mat_<float>(3, 1) <<10.99, 0.28, 0.77);//差4.04、20cm考慮3.84
    //MarkerW[2]= (cv::Mat_<float>(3, 1) <<2.62, 0.28, 2.90);
    //LX=1.75,VX=0.25,omegaZ=2.05,THZ=0.10,LY=5.0;
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0, 0.28, 2.0);
    //LX=5.00,VX=0.0,omegaZ=2.05,THZ=0.10,LY=5.0;
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<1.00, 0.28, 2.0);//差4.33、20cm考慮4.13


    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0.5, 0.28, 6);
    //MarkerW[1]= (cv::Mat_<float>(3, 1) <<0.9, 0.28, 2);

    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 3.0);
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0, 0.28, 3.0);
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0, 0.28, 1);

    //LX=5.0,VX=0,omegaZ=0,THZ=0,LY=0;
    
    //2022-01-22廊下(直進10.0m,速度0.25)直線動作実験(研究室前スタート)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.91, 0.28, 3.0);//実測値(X:9.67,Y:0.10)
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0.91, 0.28, 10.57);
    //LX=9.0,VX=0.25,omegaZ=0,THZ=0,LY=0;

    //20220317廊下(直進8.0m,速度0.25)直線動作実験(研究室前スタート)
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.87, 0.28, 3.00);//右手系(左方向X,縦方向Y,奥行きZ))
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0.87, 0.28, 6.84);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<10.38, 0.28, 10.57);
    //LX=0,VX=0,omegaZ=0,THZ=0,LY=0;
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.87, 0.28, 3.00);//右手系(左方向X,縦方向Y,奥行きZ))
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0.87, 0.28, 6.84);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<0.87,0.18,10.57);
    //LX=8.0,VX=0.25,omegaZ=0,THZ=0,LY=0;

    //20220325ゼミ室前廊下(直進2.25m,速度0.25,回転π/2度,回転速度0.20,直進5.0m,速度0.25)廊下回転動作実験未観測区間大（仮）8.48+1.90
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 3.00);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<2.62, 0.28, 2.90);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<10.38, 0.28, 2.90);
    //LX=2.25,VX=0.25,omegaZ=2.05,THZ=0.20,LY=5.0;

    //20220506廊下直進実験（走行時の深度地測定）
    //MarkerW[1]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 4.00);
    //MarkerW[2]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 2.00);
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 3.00);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0, 0.28, 5.00);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 5.00);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<0.90, 0.28, 6.00);
    //LX=5.00,VX=0,omegaZ=0,THZ=0,LY=0;

    //20220506廊下直進(長)実験（走行時の深度地測定）
    //MarkerW[1]= (cv::Mat_<float>(3, 1) <<-0.90, 0.28, 3.00);
    //MarkerW[2]= (cv::Mat_<float>(3, 1) <<-0.90, 0.28, 6.00);
    //MarkerW[3]= (cv::Mat_<float>(3, 1) <<-0.90, 0.28, 9.00);
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.90, 0.28, 12.00);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<-0.90, 0.28, 15.00);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<-0.90, 0.28, 18.10);
    //LX=8.00,VX=0.25,omegaZ=0,THZ=0,LY=8.0;
    //LX=8.00,VX=0,omegaZ=0,THZ=0,LY=0.25;

    //20220514回転運動（直進のみ）確認実験
    //MarkerW[1]= (cv::Mat_<float>(3, 1) <<0.98, 0.28, 10.99);
    //MarkerW[2]= (cv::Mat_<float>(3, 1) <<-1.15, 0.28, 2.62);
    //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0.98, 0.28, 2.62);
    //MarkerW[6]= (cv::Mat_<float>(3, 1) <<0.98, 0.28, 6.95);
    //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.98, 0.28, 2);
    //LX=5.00,VX=0.25,omegaZ=0,THZ=0,LY=0;

	ros::spin();//トピック更新待機
			
	return 0;
}