#include <ros/ros.h>
#include "string.h"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "core_msgs/line.h"

using namespace cv;
using namespace std;

Mat buffer, buffer_bottom_part, buffer_bottem_padding, buffer_top_part;
ros::Publisher pub;
int stage=-1;

void line_detect(){
  int col = buffer.cols;
  int row = buffer.rows;
  int kernel = 5;
  vector<Vec4i> linesP;
  buffer_bottom_part = buffer(Rect(Point(0,(int)row/2),Point(col,row)));
  threshold(buffer_bottom_part, buffer_bottom_part, 30, 255, THRESH_BINARY_INV);
  GaussianBlur(buffer_bottom_part, buffer_bottom_part, Size(kernel,kernel), 9, 9, BORDER_REFLECT);
  imshow("line detect blurred", buffer_bottom_part);
  waitKey(10);
  buffer_bottem_padding = buffer(Rect(Point(0,(int)row/2),Point(col,(int)row/2+(kernel-1)/2)));
  buffer_bottem_padding = Scalar(0);
  threshold(buffer_bottom_part, buffer_bottom_part, 3, 255, THRESH_BINARY);
  imshow("line detect threshold", buffer_bottom_part);
  waitKey(10);
  //Canny(buffer_bottom_part,buffer_bottom_part,100,200,3);
  //imshow("line detect canny", buffer_bottom_part);
  //GaussianBlur(buffer_bottom_part,buffer_bottom_part, Size(11,11),10,10);
  //imshow("line detect canny_blurred", buffer_bottom_part);
  //waitKey(10);
  HoughLinesP(buffer_bottom_part,linesP,1,CV_PI/90,100,100,300);

  for (size_t i=0; i<linesP.size(); i++){
    Vec4i l = linesP[i];
    line(buffer_bottom_part, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(255,0,255), 10);
  }
  // if ((int)linesP.size() > 0) stage = 1;  // line_tracing_zone
  // else stage = 0; //ball_harvesting_zone
  core_msgs::line msg;  //create a message for ball positions
  msg.stage = stage; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  pub.publish(msg);
  cout << "published! line = "<< linesP.size() << endl;
  imshow("line detect", buffer_bottom_part);
  waitKey(100);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     buffer = cv_bridge::toCvShare(msg, "mono8")->image;  //transfer the image data into buffer
  }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   line_detect(); //proceed ball detection
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "line_detect_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
   pub = nh.advertise<core_msgs::line>("/stage", 100); //setting publisher

   ros::spin(); //spin.
   return 0;
}
