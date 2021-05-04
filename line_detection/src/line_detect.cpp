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

Mat buffer, buffer_bottom_part;
ros::Publisher pub;
int stage;

void line_detect(){
  Rect bound(0,0,buffer.cols, buffer.rows);
  Rect roi(0,buffer.rows/2, buffer.cols, buffer.rows);
  buffer_bottom_part = buffer(roi&bound);
  GaussianBlur(buffer_bottom_part,buffer_bottom_part, Size(5,5),2.5, 2.5);
  threshold(buffer_bottom_part, buffer_bottom_part, 50, 255, THRESH_BINARY_INV);

  int sum = 0;
  for (int i=0; i<buffer_bottom_part.rows; i++){
    const int* r=buffer_bottom_part.ptr<int>(i);
    for (int j=0; j<buffer_bottom_part.cols; j++){
      sum += r[j];
    }
  }
  if (sum > 10) stage = 1;  // line_tracing_zone
  else stage = 0; //ball_harvesting_zone
  core_msgs::line msg;  //create a message for ball positions
  msg.stage = stage; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  pub.publish(msg);
  cout << "published! sum = "<< sum << endl;
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
   pub = nh.advertise<core_msgs::line>("/line", 100); //setting publisher

   ros::spin(); //spin.
   return 0;
}
