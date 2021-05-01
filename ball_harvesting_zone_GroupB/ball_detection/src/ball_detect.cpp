#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/ball_info.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include "geometry_msgs/Vector3.h"
#include "gazebo_msgs/ModelStates.h"

using namespace cv;
using namespace std;

Mat buffer = Mat::zeros(1920, 1080, CV_8UC3);
Mat buffer_depth = Mat::zeros(1920, 1080, CV_32F);

ros::Publisher pub;

float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};
float fball_diameter = 0.14 ;

Mat result;
Mat blue_mask;
Mat green_mask;
Mat red_mask;

float pos_x;
float pos_y;
float pos_o;

float x;
float y;
float o;

vector<float> pixel2point(int x_p2p, int y_p2p, int radius, int color) {
  Mat depth_mask = Mat::zeros(buffer_depth.rows, buffer_depth.cols, CV_8UC1);
  circle(depth_mask, Point(x_p2p, y_p2p), radius, 255, -1);

  if(color == 1){
    depth_mask = depth_mask & blue_mask;
  }
  else if(color == 2){
    depth_mask = depth_mask & red_mask;
  }
  else if(color == 3){
    depth_mask = depth_mask & green_mask;
  }

  Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(1, 1));
  erode(depth_mask, depth_mask, mask, cv::Point(-1, -1), 3);

  Scalar mean_value ,StdDev_value;
  meanStdDev(buffer_depth, mean_value ,StdDev_value, depth_mask);
  float mean_val = sum(mean_value)[0];

  Mat masked_depth = Mat::ones(buffer_depth.rows, buffer_depth.cols, buffer_depth.type())*mean_val;
  buffer_depth.copyTo(masked_depth,depth_mask);

  int min[2], max[2];
  double min_p2p, max_p2p;
  minMaxIdx(masked_depth, &min_p2p, &max_p2p, min, max);
  circle(result,Point(min[1],min[0]),3,Scalar(0,0,0),-1);

  vector<float> position;
  if(max_p2p-min_p2p >= 0.07){
    position.push_back(0);
    return position;
  }

  float xi, yi, u, v, Xc, Yc, Zc;
  xi = (float)min[1];
  yi = (float)min[0];
  u = (xi-intrinsic_data[2])/intrinsic_data[0];
  v = (yi-intrinsic_data[5])/intrinsic_data[4];
  Zc = min_p2p;
  Xc = u*Zc ;
  Yc = v*Zc ;
  Xc = roundf(Xc * 1000) / 1000;
  Yc = roundf(Yc * 1000) / 1000;
  Zc = roundf(Zc * 1000) / 1000 + 0.222;
  /*
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc+0.222);
  */
  position.push_back(x + cosf(o)*Zc + sinf(o)*Xc);
  position.push_back(Yc);
  position.push_back(y + sinf(o)*Zc - cosf(o)*Xc);
  return position;
}

void ball_detect(){
  core_msgs::ball_info msg;
  msg.pose.push_back(x);
  msg.pose.push_back(y);
  msg.pose.push_back(o);

  buffer.copyTo(result);

  Mat hsv;
  cvtColor(buffer, hsv, CV_RGB2HSV);

  Mat red1_mask,red2_mask;

  inRange(hsv,Scalar(110,50,0),Scalar(120,255,255),blue_mask);
  inRange(hsv,Scalar(50,50,0),Scalar(70,255,255),green_mask);
  inRange(hsv,Scalar(0,50,0),Scalar(10,255,255),red1_mask);
  inRange(hsv,Scalar(160,50,0),Scalar(179,255,255),red2_mask);
  red_mask = red1_mask | red2_mask;

  Mat blue_mask_GB;
  Mat green_mask_GB;
  Mat red_mask_GB;

  GaussianBlur(blue_mask, blue_mask_GB, cv::Size(5,5),0);
  GaussianBlur(green_mask, green_mask_GB, cv::Size(5,5),0);
  GaussianBlur(red_mask, red_mask_GB, cv::Size(5,5),0);

  vector<Vec3f> red_circles,green_circles,blue_circles;
  HoughCircles(red_mask_GB,red_circles,HOUGH_GRADIENT, 1, 100, 50, 24, 0,0);
  HoughCircles(green_mask_GB,green_circles,HOUGH_GRADIENT, 1, 100, 50, 24, 0,0);
  HoughCircles(blue_mask_GB,blue_circles,HOUGH_GRADIENT, 1, 70, 100, 24, 0,0);

  Vec3f params;
  float cx,cy,r;
  cout<<"red_circles.size="<<red_circles.size()<<endl;
  cout<<"green_circles.size="<<green_circles.size()<<endl;
  cout<<"blue_circles.size="<<blue_circles.size()<<endl;

  int ball_number = 0;

  for(int bi=0;bi<blue_circles.size();bi++){
    params = blue_circles[bi];
    cx=cvRound(params[0]);
    cy=cvRound(params[1]);
    r=cvRound(params[2]);

    Point blue_center(cx,cy);
    circle(result,blue_center,r,Scalar(0,0,255),1);

    vector<float> ball_pos_b;
    ball_pos_b = pixel2point(cx, cy, r, 1);

    if(!ball_pos_b[0]){
      continue;
    }
    /*
    float isx = ball_pos_b[0];
    float isy = ball_pos_b[1];
    float isz = ball_pos_b[2];


    string sx = to_string(isx);
    string sy = to_string(isy);
    string sz = to_string(isz);

    string text_b;
    text_b = "x:" + sx +", y:" + sy + ", z:" + sz;
    putText(result, text_b, blue_center, 1, 2, Scalar(0,0,255), 1);
    */

    msg.img_x.push_back(ball_pos_b[0]);
    msg.img_y.push_back(ball_pos_b[2]);
    msg.color.push_back(1);
    ball_number++;
  }

  for(int ri=0;ri<red_circles.size();ri++){
    params = red_circles[ri];
    cx=cvRound(params[0]);
    cy=cvRound(params[1]);
    r=cvRound(params[2]);

    Point red_center(cx,cy);
    circle(result,red_center,r,Scalar(255,0,0),2);

    vector<float> ball_pos_r;
    ball_pos_r = pixel2point(cx, cy, r, 2);

    if(!ball_pos_r[0]){
      continue;
    }

    /*
    float isx = ball_pos_r[0];
    float isy = ball_pos_r[1];
    float isz = ball_pos_r[2];

    string sx = to_string(isx);
    string sy = to_string(isy);
    string sz = to_string(isz);

    string text_r;
    text_r = "x:" + sx +", y:" + sy + ", z:" + sz;
    putText(result, text_r, red_center, 1, 2, Scalar(255,0,0), 1);
    */

    msg.img_x.push_back(ball_pos_r[0]);
    msg.img_y.push_back(ball_pos_r[2]);
    msg.color.push_back(2);
    ball_number++;
  }

  for(int gi=0;gi<green_circles.size();gi++){
    params = green_circles[gi];
    cx=cvRound(params[0]);
    cy=cvRound(params[1]);
    r=cvRound(params[2]);

    Point green_center(cx,cy);
    circle(result,green_center,r, Scalar(0,255,0),2);

    vector<float> ball_pos_g;
    ball_pos_g = pixel2point(cx, cy, r, 3);

    /*
    float isx = ball_pos_g[0];
    float isy = ball_pos_g[1];
    float isz = ball_pos_g[2];

    string sx = to_string(isx);
    string sy = to_string(isy);
    string sz = to_string(isz);

    string text_g;
    text_g = "x:" + sx +", y:" + sy + ", z:" + sz;
    putText(result, text_g, green_center, 1, 2, Scalar(0,255,0), 1);
    */

    msg.img_x.push_back(ball_pos_g[0]);
    msg.img_y.push_back(ball_pos_g[2]);
    msg.color.push_back(3);
    ball_number++;
  }

  msg.number = ball_number;

  //cv::imshow("view", result);
  cv::waitKey(1);
  pub.publish(msg);
}

void odom_Callback(const gazebo_msgs::ModelStates::ConstPtr& odometry)
{
    pos_x = odometry->pose[17].position.x;
    pos_y = odometry->pose[17].position.y;
    float qx, qy, qz, qw;
    qx = odometry->pose[17].orientation.x;
    qy = odometry->pose[17].orientation.y;
    qz = odometry->pose[17].orientation.z;
    qw = odometry->pose[17].orientation.w;
    pos_o = atan2(2.0*(qw*qz+qx*qy),1.0-2.0*(qy*qy+qz*qz));
    //cout<<pos_x<<" , "<<pos_y<<" , "<<pos_o<<endl;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg){
  x = pos_x;
  y = pos_y;
  o = pos_o;
  try{
    cv_bridge::CvImageConstPtr depth_img_cv;
    depth_img_cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img_cv->image.convertTo(buffer_depth, CV_32F);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    buffer = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ball_detect_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  //ros::Subscriber sub1 = nh.subscribe<geometry_msgs::Vector3>("/odometry", 1, odom_Callback);
  ros::Subscriber sub1 = nh.subscribe("/gazebo/model_states", 1, odom_Callback);
  image_transport::Subscriber sub_depth = it.subscribe("/camera/depth/image_raw", 1, depthCallback);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

  pub = nh.advertise<core_msgs::ball_info>("/position", 1);

  while(ros::ok){
    ros::spinOnce();
    ball_detect();
  }



  //ros::spin();
  return 0;
}
