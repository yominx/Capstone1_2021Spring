#include <ros/ros.h>
#include "string.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include "core_msgs/ball_position.h"
#include "core_msgs/goal_position.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <math.h>

using namespace cv;
using namespace std;

Mat buffer, buffer_depth;
ros::Publisher pubBall;
ros::Publisher pubGoal;
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

bool goal_check(int row, int col, int r, Mat img){
  // return (img.at<uchar>(row-r+1,col) > 45 && img.at<uchar>(row-r+1,col) < 75 && img.at<uchar>(row,col) > 45 && img.at<uchar>(row,col) < 75);
  return (img.at<uchar>(row,col) > 40 || img.at<uchar>(row-r/2,col) > 40);
}

bool filtering1(int row, int col, int r, Mat img){
  return (isBlack(row,col-r/2,img) && isBlack(row,col+r/2,img) && isBlack(row,col,img));
}

bool filtering2(int row, int col, float r, float focalLen, Mat img, float& dist){
  int rFloor = (int)cvFloor(r);
  int targetPoint;
  float targetPointDist, angle;
  if (isBlack(row,col,img)){ // blocked by pillar or scooper
    angle = atan((2.5*(col-319.5)/320)/4.6621);
    // if (isBlack(row,col-rFloor,img))
    targetPoint = (isBlack(row,col-rFloor,img)) ? col+rFloor : col-rFloor;
    targetPointDist = buffer_depth.at<float>(row,targetPoint)/cos(angle);
  }
  else {
    targetPoint = row - rFloor;
    targetPointDist = buffer_depth.at<float>(targetPoint,col) / cos(angle);
  }
  float centerDist = sqrt(targetPointDist*targetPointDist + dHeight*dHeight);
  float r_pred = RADIUS*FOCAL_LENGTH/centerDist;
  // Point center(col,row);
  // circle(buffer,center,r_pred,Scalar(255,0,0),2);
  cout << "(r_pred, r_measured)= (" << r_pred << ", " << r << ")" << endl;
  if (abs(r_pred-r) < THRESHOLD){
    dist = targetPointDist;
    return true;
  }
  return false;
}

bool filtering3(int row, int col, int r, Mat img, float& dist){ // too close to detect
  if (isBlack(row,col-r,img) || isBlack(row,col+r,img)) return false;
  int lWalk=0, rWalk=0, uWalk=0;
  while (!isBlack(row,col-r-(lWalk+1),img)){
    ++lWalk;
  }
  while (!isBlack(row,col+r+rWalk+1,img)){
    ++rWalk;
  }
  if ((!isBlack(row-r,col,img)) && abs(lWalk-rWalk) < THRESHOLD){
    while(!isBlack(row-r-(uWalk+1),col,img)){
      ++uWalk;
    }
    dist = img.at<uchar>(row-r-uWalk,col);
    return true;
  }
  return false;
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
      continue;
    }
    f = sqrt(pow(FOCAL_LENGTH,2)+pow((WIDTH/2.-col),2));
    angle = atan((2.5*(col-319.5)/320)/4.6621);
    distance = (buffer_depth.at<float>(row, col) + RADIUS) / cos(angle);
    cout << "distance = " << distance <<endl;
    //r_pred = RADIUS*f/sqrt(distance*distance + dHeight*dHeight);
    r_pred = RADIUS*f/distance;
    cout << "(r_measured, r_pred) : (" << r << ", " << r_pred << ")" << endl;
    if (abs(r_pred-r) < r/7){
      circle[0] = col;
      circle[1] = row;
      circle[2] = r;
      circle[3] = distance;
      filtered.push_back(circle);
      continue;
    }

    // if (abs(buffer_depth.at<float>(row,col-r*0.7) - buffer_depth.at<float>(row,col+r*0.7)) > RADIUS){ // blocked by another ball or pillar or scooper
    //   if (filtering2(row,col,r,f,img,distance)){
    //     circle[0] = col;
    //     circle[1] = row;
    //     circle[2] = r;
    //     circle[3] = distance;
    //     filtered.push_back(circle);
    //     continue;
    //   }
    // }

    // if (row >= HEIGHT-1){  // right before harvesting the ball
    //   if (filtering3(row,col,cvFloor(circle[2]),img, distance)){
    //     circle[0] = col;
    //     circle[1] = row;
    //     circle[2] = r;
    //     circle[3] = distance;
    //     filtered.push_back(circle);
    //     continue;
    //   }
    // }
    // cout << "(r,c): (" <<row << ", " << col << "), rad:" << r << ", f: " << f << ", dist: " << distance << endl;
    // cout << "r_predictd: " << r_pred << ", r_pixel: " << r << endl;

  }
  return filtered;
}

void ball_detect(){
     Mat hsv, gray;  //assign a memory to save the edge images
     Mat hsvCh[3];
     Mat frame;  //assign a memory to save the images
     //Mat mask,mask1, mask2;
     // cv::imshow("raw_rgb", buffer);
     // waitKey(10);
     cvtColor(buffer, hsv, CV_BGR2HSV);
     split(hsv, hsvCh);
     // cv::imshow("h", hsvCh[0]);
     // waitKey(10);
     // cv::imshow("s", hsvCh[1]);
     // waitKey(10);
     // cv::imshow("v", hsvCh[2]);
     // waitKey(10);

     GaussianBlur(hsvCh[1],hsvCh[1], Size(5,5), 2.5, 2.5);
     // cv::imshow("Gaussian Blurred", hsvCh[0]);
     // waitKey(10);
     threshold(hsvCh[1], gray, 100, 255, THRESH_BINARY);
     // cv::imshow("Binary", gray);
     // waitKey(10);
     // cv::imshow("h", hsvCh[0]);
     // waitKey(10);
     vector<Vec3f> circles; //assign a memory to save the result of circle detection
     vector<Vec4f> filteredCircles; //<row,col,r,dist>
     HoughCircles(gray,circles,HOUGH_GRADIENT, 1, gray.rows/30, 30, 10, 7, 200); //proceed circle detection
     //Circles (Cx, Cy, r)

     //before Filtering
     for(int k=0,i=0;k<circles.size();k++){
         Vec3f params = circles[k];  //the information of k-th circle
         float c_c=cvRound(params[0]);  //x position of k-th circle
         float c_r=cvRound(params[1]);  //y position
         float r=cvRound(params[2]); //radius

         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         circle(buffer,center,r,Scalar(255,0,255),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
     }
     filteredCircles = filtering(circles, gray);
     Vec4f params; //assign a memory to save the information of circles
     int c_c,c_r,r;
     int nBalls = filteredCircles.size();
     int goalIndex = -1;
     core_msgs::ball_position msgBall;  //create a message for ball positions
     core_msgs::goal_position msgGoal;

     for (int i=0; i<filteredCircles.size(); i++){
        params = filteredCircles[i];
        c_c = (int)cvRound(params[0]);
        c_r = (int)cvRound(params[1]);
        r = (int)cvFloor(params[2]);
        if (goal_check(c_r,c_c,r, hsvCh[0])){
          nBalls -= 1;
          goalIndex = i;
          break;
        }
     }

     msgBall.size = nBalls; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msgBall.angle.resize(nBalls);  //adjust the size of array
     msgBall.dist.resize(nBalls);  //adjust the size of array

     for(int k=0,i=0;k<filteredCircles.size();k++){
         params = filteredCircles[k];  //the information of k-th circle
         c_c=(int)cvRound(params[0]);  //x position of k-th circle
         c_r=(int)cvRound(params[1]);  //y position
         r=(int)cvRound(params[2]); //radius

         // 원 출력을 위한 원 중심 생성
         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         // cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
         // cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);
         if (k == goalIndex){
           circle(buffer,center,r,Scalar(0,255,0),3); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
           msgGoal.angle = atan((2.5*(c_c-319.5)/320)/4.6621); // [rad]
           // msgGoal.angle = atan((c_c-319.5)/320*tan(FOV)); // [rad]
           msgGoal.dist = params[3];
           pubGoal.publish(msgGoal);
           cout << "Goal(row,col) : (" << c_r << ", " << c_c << "), dist= " << msgGoal.dist << ", angle= " << msgGoal.angle << endl;
           continue;
         }
         circle(buffer,center,r,Scalar(0,0,255),3); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
         // msgBall.angle[i]=atan((c_c-319.5)/320*tan(FOV));  //[rad]
         msgBall.angle[i] = atan((2.5*(c_c-319.5)/320)/4.6621); // [rad]
         msgBall.dist[i]=params[3];   //[m]
         cout << i+1 <<"/" << nBalls << " Ball(row,col) : (" <<c_r <<", " << c_c << "),dist= " << msgBall.dist[i] << ", angle= " << msgBall.angle[i] << endl;
         ++i;

     }
     cout << endl;
     pubBall.publish(msgBall);  //publish a message
     // cv::imshow("view", buffer);  //show the image with a window
     // cv::waitKey(1);
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
   if (!buffer_depth.empty())
   {
     ball_detect();
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


int main(int argc, char **argv)
{
   ros::init(argc, argv, "ball_detect_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub_rgb = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
   image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthCallback);
   pubBall = nh.advertise<core_msgs::ball_position>("/ball_position", 100); //setting publisher
   pubGoal = nh.advertise<core_msgs::goal_position>("/goal_position", 100); //setting publisher

   ros::spin(); //spin.
   return 0;
}
