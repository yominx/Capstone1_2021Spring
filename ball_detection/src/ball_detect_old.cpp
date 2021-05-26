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

bool goal_check(int r, int c){
    Mat bgrCh[3];
    split(buffer, bgrCh);
    if ((int)bgrCh[1].at<uchar>(r,c) > 100) return true;
    return false;
}

vector<Vec3f> ransac(vector<Vec3f> circles){
  vector<Vec3f> cRansac;
  int size = circles.size();
  Vec3f circle;
  float cC, cR;
  for (int i=0; i<size; i++){
    circle = circles[i];
    cC = cvRound(circle[0]);
    cR = cvRound(circle[1]);
    // Scalar mean_value = mean(buffer_depth, gray);
    // float mean_val = sum(mean_value)[0];
    // cout << "mean_val: "<< mean_val<<endl;
  }
  return circles;
}

void ball_detect(){
     Mat hsv, gray;  //assign a memory to save the edge images
     Mat hsvCh[3];
     Mat frame;  //assign a memory to save the images
     //Mat mask,mask1, mask2;
     // cv::imshow("raw_rgb", buffer);
     // waitKey(10);
     cvtColor(buffer, hsv, CV_RGB2HSV);
     split(hsv, hsvCh);
     // cv::imshow("h", hsvCh[0]);
     // waitKey(10);
     GaussianBlur(hsvCh[0],hsvCh[0], Size(5,5), 2.5, 2.5);
     // cv::imshow("Gaussian Blurred", hsvCh[0]);
     // waitKey(10);
     threshold(hsvCh[0], gray, 50, 255, THRESH_BINARY);
     // cv::imshow("Binary", gray);
     // waitKey(10);
     vector<Vec3f> circles; //assign a memory to save the result of circle detection
     HoughCircles(gray,circles,HOUGH_GRADIENT, 1, gray.rows/20, 100, 15, 0,0); //proceed circle detection
     //Circles (Cx, Cy, r)
     Vec3f params; //assign a memory to save the information of circles
     float c_c,c_r,r;
     int nBalls = circles.size();
     int goalIndex = -1;
     core_msgs::ball_position msgBall;  //create a message for ball positions
     core_msgs::goal_position msgGoal;

     for (int i=0; i<circles.size(); i++){
        params = circles[i];
        c_c = cvRound(params[0]);
        c_r = cvRound(params[1]);
        if (goal_check(c_r,c_c)){
          nBalls -= 1;
          goalIndex = i;
          break;
        }
     }

     msgBall.size = nBalls; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msgBall.angle.resize(nBalls);  //adjust the size of array
     msgBall.dist.resize(nBalls);  //adjust the size of array

 	   // visualization_msgs::Marker ball_list;  //declare marker
	   // ball_list.header.frame_id = "/camera_link";  //set the frame
	   // ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
	   // ball_list.ns = "balls";   //name of markers
	   // ball_list.action = visualization_msgs::Marker::ADD;
	   // ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
	   // ball_list.pose.position.y=0;
	   // ball_list.pose.position.z=0;
	   // ball_list.pose.orientation.x=0;
	   // ball_list.pose.orientation.y=0;
	   // ball_list.pose.orientation.z=0;
	   // ball_list.pose.orientation.w=1.0;
     //
	   // ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
	   // ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker
     //
	   // double radius = 0.15;
     // ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
     // ball_list.scale.y=radius;
     // ball_list.scale.z=radius;

     for(int k=0,i=0;k<circles.size();k++){
         params = circles[k];  //the information of k-th circle
         c_c=cvRound(params[0]);  //x position of k-th circle
         c_r=cvRound(params[1]);  //y position
         r=cvRound(params[2]); //radius

         // 원 출력을 위한 원 중심 생성
         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         circle(buffer,center,r,Scalar(0,0,255),10); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
         // cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
         // cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);
         if (k == goalIndex){
           msgGoal.angle = atan((2.5*(c_c-319.5)/320)/4.6621); // [rad]
           //camera : |theta|=atac( (2.5*{pixel}/{width/2}) / 4.6621 )  // [m]
           msgGoal.dist = buffer_depth.at<float>(c_r,c_c);
           pubGoal.publish(msgGoal);
           cout << "Goal(row,col) : (" << c_r << ", " << c_c << "), dist= " << msgGoal.dist << ", angle= " << msgGoal.angle << endl;
           continue;
         }

         msgBall.angle[i]=atan((2.5*(c_c-319.5)/320)/4.6621);  //[rad]
         msgBall.dist[i]=buffer_depth.at<float>(c_r,c_c);   //[m]
         cout << i <<"/" << nBalls << " Ball(row,col) : (" <<c_r <<", " << c_c << "),dist= " << msgBall.dist[i] << ", angle= " << msgBall.angle[i] << endl;
         i++;
         // geometry_msgs::Point p;
	       // p.x = cx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
	       // p.y = cy;
	       // p.z = 0.1;
	       // ball_list.points.push_back(p);
         // std_msgs::ColorRGBA c;
	       // c.r = 0.0;  //set the color of the balls. You can set it respectively.
	       // c.g = 1.0;
	       // c.b = 0.0;
	       // c.a = 1.0;
	       // ball_list.colors.push_back(c);
     }
     cout << endl;
     cv::imshow("view", buffer);  //show the image with a window
     cv::waitKey(1);
     pubBall.publish(msgBall);  //publish a message
     //pub_markers.publish(ball_list);  //publish a marker message

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
   // ball_detect(); //proceed ball detection
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "ball_detect_node_old"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub_rgb = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
   image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthCallback);
   pubBall = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
   pubGoal = nh.advertise<core_msgs::goal_position>("/goal_position", 100); //setting publisher

   //pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

   ros::spin(); //spin.
   return 0;
}
