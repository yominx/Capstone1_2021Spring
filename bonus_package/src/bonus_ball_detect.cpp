#include <ros/ros.h>
#include "string.h"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include "core_msgs/ball_position.h"
#include "core_msgs/goal_position.h"
#include "std_msgs/Float32MultiArray.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
// #define DEBUG
// #define DEBUG_IMG

using namespace cv;
using namespace std;
int waytype;
Mat buffer, buffer_depth, original_img;
Mat ball_img, goal_img;
ros::Publisher pubBall;
ros::Publisher pubGoal;
ros::Publisher pubClosest;
//ros::Publisher pub_markers;

float FOCAL_LENGTH = 589.37;
int WIDTH = 639;
int HEIGHT = 479;

float RADIUS = 0.15/2; //[m]
float THRESHOLD = 3;
float FOV = 28.5*M_PI/180;
float dHeight = 0.178 - RADIUS;

bool isBlack(int row, int col, Mat img){
  return img.at<uchar>(row,col)<100;
}

void print(int index, int row, int col, int phase, bool flag, float detect_r, float pred_r, float dist)
{
  char str[30];
  if (flag){
    return;
    snprintf(str, sizeof(str), "[Phase %d] SUCCESS (%d-th)", phase, index);
    putText(buffer, str, Point(col-2*detect_r, row), 1, 1, Scalar(0, 0, 255), 1, 8);
    cout << "[Phase " << phase << "] (" << index << "-th) SUCCESS (r_detect, r_pred): (" << detect_r << ", " << pred_r << ") distance: " << dist  << endl;
  }
  else {
    snprintf(str, sizeof(str), "[Phase %d] filterted (%d-th)", phase, index);
    putText(buffer, str, Point(col-2*detect_r, row), 1, 1, Scalar(255, 0, 0), 1, 8);
    cout << "[Phase " << phase << "] (" << index << "-th) filtered (r_detect, r_pred): (" << detect_r << ", " << pred_r << ") distance: " << dist<< endl;
  }
}

bool filtering1(int row, int col, int r, Mat img){
  return (isBlack(row,col-r/2,img) && isBlack(row,col+r/2,img) && isBlack(row,col,img));
}

bool filtering2(int row, int col, float r, Mat img){
  int targetPoint;
  int rFloor = (int)cvFloor(r);
  if (isBlack(row,col,img)){
    targetPoint = (isBlack(row,col-rFloor,img)) ? col + rFloor : col - rFloor;
    if (buffer_depth.at<float>(row,col)-buffer_depth.at<float>(row,targetPoint) <= RADIUS) {return false;}
  }
  else {
    targetPoint = row - rFloor;
    if (buffer_depth.at<float>(targetPoint+THRESHOLD,col)-buffer_depth.at<float>(row,col) < 2*RADIUS) {return false;}
  }
  return true;
}

bool filtering3(int row, int col){ // too close to detect
  return (buffer_depth.at<float>(row,col) < 0.5) ? true : false;
}

bool filtering4(int row, int col, Mat img)
{
  return (!isBlack(HEIGHT, WIDTH/2, img));
}

int findTop(int row, int col, Mat img)
{
  int r = row;
  while (!isBlack(r-1,col,img)){
    r -= 1;
  }
  return r;
}

vector<Vec4f> filtering(vector<Vec3f> circles, Mat img){
  vector<Vec4f> filtered; // 0:col, 1:radius, 2:distance
  Vec4f circle;
  int size = circles.size();
  int col, row;
  float r, r_pred, distance, f, angle;
  for (int i=0; i<size; i++){
    col = (int)cvRound(circles[i][0]);
    row = (int)cvRound(circles[i][1]);
    r = circles[i][2];
    if (filtering1(row,col,r,img)){ // in case of detecing floor as a ball
#ifdef DEBUG
      print(i, row, col, 1, false, 0, 0, 0);
#endif
      continue;
    }

    f = sqrt(pow(FOCAL_LENGTH,2)+pow((WIDTH/2.-col),2));
    angle = atan((2.5*(col-319.5)/320)/4.6621);
    distance = (buffer_depth.at<float>(row, col) + RADIUS) / cos(angle);
    r_pred = RADIUS*f/distance;

    if (filtering2(row,col,r,img)){ // block by pillar of other balls
      int rFloor = (int)cvFloor(r);
      int targetPoint;
      float targetPointDist, angle;
      if (isBlack(row,col,img)){ // blocked by pillar or scooper
        targetPoint = (isBlack(row,col-rFloor+THRESHOLD,img)) ? col+rFloor-THRESHOLD : col-rFloor+THRESHOLD;
        angle = atan((col-319.5)/320*tan(FOV));
        targetPointDist = buffer_depth.at<float>(row,targetPoint)/cos(angle);
      }
      else {
        targetPoint = row-rFloor+THRESHOLD;
        angle = atan((2.5*(col-319.5)/320)/4.6621);
        targetPointDist = buffer_depth.at<float>(targetPoint,col) / cos(angle);
      }
      float r_pred = RADIUS*FOCAL_LENGTH/targetPointDist;
      if (abs(r_pred-r) < r/5){
        circle[0] = col;
        circle[1] = row;
        circle[2] = r;
        circle[3] = targetPointDist;
        filtered.push_back(circle);
#ifdef DEBUG
        print(i, row, col, 2, true, r, r_pred, targetPointDist);
      }
      else {
        print(i, row, col, 2, false, r, r_pred, targetPointDist);
#endif
      }
      continue;
    }
    if (filtering3(row,col)){
      continue;
    }
    if (filtering4(HEIGHT-5,col,img)){
      int r = findTop(HEIGHT-5,col,img);
      float dist = buffer_depth.at<float>(r+3,col);
      circle[0] = col;
      circle[1] = r;
      circle[2] = RADIUS*f/dist;
      circle[3] = dist;
      filtered.push_back(circle);
      continue;
    }
    if (abs(r_pred-r) < r/5){
      circle[0] = col;
      circle[1] = row;
      circle[2] = r;
      circle[3] = distance;
      filtered.push_back(circle);
#ifdef DEBUG
      print(i, row, col, 3, true, r, r_pred, distance);
#endif
      continue;
    }
#ifdef DEBUG
    print(i,row,col,3,false, r, r_pred, distance);
#endif
  }
  return filtered;
}

void sort(vector<Vec4f>& circles)
{
  float keyVal, targetVal;
  int j, tmp0, tmp1, tmp2, tmp3;
  for (int i=1; i<circles.size(); i++){
    tmp0 = circles[i][0];
    tmp1 = circles[i][1];
    tmp2 = circles[i][2];
    tmp3 = circles[i][3];
    keyVal = circles[i][3];
    j=i-1;
    targetVal = circles[j][3];
    while (j >= 0 && targetVal < keyVal){
      circles[j+1] = circles[j];
      j -= 1;
      targetVal = circles[j][3];
    }
    circles[j][0] = tmp0;
    circles[j][1] = tmp1;
    circles[j][2] = tmp2;
    circles[j][3] = tmp3;
  }
}
void ball_detect(){
     Mat hsv;  //assign a memory to save the edge images
     Mat hsvCh[3];
     cvtColor(buffer, hsv, CV_BGR2HSV);
     split(hsv, hsvCh);
     hsvCh[0].copyTo(goal_img);
     inRange(goal_img, Scalar(55), Scalar(65), goal_img);
     inRange(hsvCh[0], Scalar(0), Scalar(10), hsvCh[0]);
     bitwise_and(hsvCh[0],hsvCh[1],hsvCh[0]);
     GaussianBlur(hsvCh[0],hsvCh[0], Size(5,5), 2.5, 2.5);
     threshold(hsvCh[0], ball_img, 100, 255, THRESH_BINARY);
#ifdef DEBUG_IMG
        imshow("buffer", buffer);
        waitKey(10);
        imshow("ball", ball_img);
        waitKey(10);
        imshow("goal", goal_img);
        waitKey(10);
#endif
     vector<Vec3f> circlesB, circlesG; //assign a memory to save the result of circle detection
     vector<Vec4f> filteredCirclesB, filteredCirclesG; //<row,col,r,dist>
     HoughCircles(ball_img,circlesB,HOUGH_GRADIENT, 1, ball_img.rows/30, 30, 10, 7, 200); //proceed circle detection
     HoughCircles(goal_img,circlesG,HOUGH_GRADIENT, 1, goal_img.rows/30, 30, 10, 7, 200); //proceed circle detection

     //Circles (Cx, Cy, r)
#ifdef DEBUG
     // for(int k=0;k<circlesB.size();k++){
     //     Vec3f params = circlesB[k];  //the information of k-th circle
     //     float c_c=cvRound(params[0]);  //x position of k-th circle
     //     float c_r=cvRound(params[1]);  //y position
     //     float r=cvRound(params[2]); //radius
     //     Point center(c_c,c_r);  //declare a Point Point(coloum, row)
     //     circle(buffer,center,r,Scalar(255,0,255),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
     //   }
     for(int k=0;k<circlesG.size();k++){
         Vec3f params = circlesG[k];  //the information of k-th circle
         float c_c=cvRound(params[0]);  //x position of k-th circle
         float c_r=cvRound(params[1]);  //y position
         float r=cvRound(params[2]); //radius
         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         circle(buffer,center,r,Scalar(255,0,255),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
     }
#endif
     filteredCirclesB = filtering(circlesB, ball_img);
     filteredCirclesG = filtering(circlesG, goal_img);
     Vec4f params; //assign a memory to save the information of circles
     int c_c,c_r,r;
     core_msgs::ball_position msgBall;  //create a message for ball positions
     core_msgs::goal_position msgGoal;
     std_msgs::Float32MultiArray msgClosest;
     int nBalls = filteredCirclesB.size();
     msgBall.size = nBalls; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msgBall.angle.resize(nBalls);  //adjust the size of array
     msgBall.dist.resize(nBalls);  //adjust the size of array
     float minDist = 9999., minAngle = 0.;
     for(int i=0;i<nBalls;i++){
         params = filteredCirclesB[i];  //the information of k-th circle
         c_c=(int)cvRound(params[0]);  //x position of k-th circle
         c_r=(int)cvRound(params[1]);  //y position
         r=(int)cvRound(params[2]); //radius
         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         circle(buffer,center,r,Scalar(0,0,255),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
         msgBall.angle[i] = (float)atan((c_c-319.5)/320*tan(FOV));  //[rad]
         msgBall.dist[i]=params[3];   //[m]
         if (params[3] < minDist) {
           minDist = params[3];
           minAngle = atan((c_c-319.5)/320*tan(FOV));
         }
#ifdef DEBUG
         cout << "[Ball] Distance" << msgBall.dist[i] << ", Angle= " << msgBall.angle[i] << endl;
         cout << endl;
#endif
     }
     pubBall.publish(msgBall);  //publish a message
     msgClosest.data.clear();
     if (waytype != 2 && (!fabs(minDist-9999)<1000)){
     msgClosest.data.push_back(minDist);
     msgClosest.data.push_back(minAngle);
     pubClosest.publish(msgClosest);
     }
     nBalls = filteredCirclesG.size();
     cout << "-------------NGOALS---- " << nBalls << endl;
     msgGoal.size = nBalls; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msgGoal.angle.resize(nBalls);  //adjust the size of array
     msgGoal.dist.resize(nBalls);  //adjust the size of array
     minDist = 9999., minAngle = 0.;
     for(int i=0;i<nBalls;i++){
         params = filteredCirclesG[i];  //the information of k-th circle
         c_c=(int)cvRound(params[0]);  //x position of k-th circle
         c_r=(int)cvRound(params[1]);  //y position
         r=(int)cvRound(params[2]); //radius
         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         circle(buffer,center,r,Scalar(0,255,0),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
         msgGoal.angle[i]=atan((c_c-319.5)/320*tan(FOV));  //[rad]
         msgGoal.dist[i]=params[3];   //[m]
         if (params[3] < minDist) {
           minDist = params[3];
           minAngle = atan((c_c-319.5)/320*tan(FOV));
         }
#ifdef DEBUG
         cout << "[Ball] Distance" << msgBall.dist[i] << ", Angle= " << msgBall.angle[i] << endl;
         cout << endl;
#endif
     }
     pubGoal.publish(msgGoal);  //publish a message
     msgClosest.data.clear();
     if (waytype == 2 && (!fabs(minDist-9999))<1000){
     msgClosest.data.push_back(minDist);
     msgClosest.data.push_back(minAngle);
     pubClosest.publish(msgClosest);
     }
#ifdef DEBUG_IMG
     cv::imshow("view", buffer);  //show the image with a window
     cv::waitKey(1);
#endif
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     buffer_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
     buffer_depth.convertTo(buffer_depth, CV_32F, 0.001);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
   }
}

void target_Callback(const geometry_msgs::Vector3::ConstPtr& waypoint) {
	waytype = waypoint->z;
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "bonus_ball_detect"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub_rgb = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
   image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthCallback);
   ros::Subscriber sub_target = nh.subscribe<geometry_msgs::Vector3>("/waypoint", 1000, target_Callback);
   pubBall = nh.advertise<core_msgs::ball_position>("/ball_position", 10); //setting publisher
   pubGoal = nh.advertise<core_msgs::goal_position>("/goal_position", 10); //setting publisher
   pubClosest = nh.advertise<std_msgs::Float32MultiArray>("/closest_point", 1);

   ros::Rate loop_rate(10);
   while (ros::ok()){
     cout << "here" << endl;
     if(!buffer.empty() && !buffer_depth.empty()){
       ball_detect();
     }
     loop_rate.sleep();
     ros::spinOnce();
   }
   return 0;
}
