#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/ball_info.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

using namespace cv;
using namespace std;

Mat buffer;
Mat buffer_depth;
ros::Publisher pub;
/*
ros::Publisher pub_r;
ros::Publisher pub_g;
ros::Publisher pub_b;
ros::Publisher pub_markers;
*/


// Initialization of variable for camera calibration paramters.
// You should change this if you changed the size of the image.
float intrinsic_data[9] = {1206.8897719532354, 0.0, 960.5, 0.0, 1206.8897719532354, 540.5, 0.0, 0.0, 1.0};
//float intrinsic_data[9] = {402.2965907, 0.0, 320.5, 0.0, 402.2965907, 180.5, 0.0, 0.0, 1.0};
float distortion_data[5] = {0, 0, 0, 0, 0};
float fball_diameter = 0.14 ; // Initialization of variable for dimension of the target(real ball diameter by meter)

// Declaration of functions that calculates the ball position from pixel position.
vector<float> pixel2point(int x_int, int y_int, int radius) {
  vector<float> position;
  float x, y, u, v, Xc, Yc, Zc;
  x = (float)x_int;//.x;// .at(0);
  y = (float)y_int;//.y;//
  u = (x-intrinsic_data[2])/intrinsic_data[0];
  v = (y-intrinsic_data[5])/intrinsic_data[4];
  //Zc = (intrinsic_data[0]*0.239)/(y-intrinsic_data[5]) ;
  Zc = 0.239*tan(atan(1.0)*2 - atan((y-intrinsic_data[5])/intrinsic_data[0]) + 1*atan(1.0)/45.0);
  Xc = u*Zc ;
  Yc = v*Zc ;
  Xc = roundf(Xc * 1000) / 1000;
  Yc = roundf(Yc * 1000) / 1000;
  Zc = roundf(Zc * 1000) / 1000;
  position.push_back(Xc);
  position.push_back(Yc);
  position.push_back(Zc);
  return position;
}

void ball_detect(){
  Mat gray;  //assign a memory to save the edge images
  Mat frame;  //assign a memory to save the images
  Mat mask,mask1, mask2;
  Mat result;

  //resize(buffer,buffer,Size(640,360),0,0,INTER_LINEAR);

  buffer.copyTo(result);

  cvtColor(buffer, gray, CV_RGB2GRAY);
  gray=~gray;

  Mat hsv;
  Mat hsv_split[3];
  Mat sat_mask;
  Mat only_balls;

  cvtColor(buffer, hsv, CV_RGB2HSV);

  Mat blue_mask;
  Mat green_mask;
  Mat red_mask,red1_mask,red2_mask;

  inRange(hsv,Scalar(110,50,0),Scalar(120,255,255),blue_mask);
  inRange(hsv,Scalar(50,50,0),Scalar(70,255,255),green_mask);
  inRange(hsv,Scalar(0,50,0),Scalar(10,255,255),red1_mask);
  inRange(hsv,Scalar(160,50,0),Scalar(179,255,255),red2_mask);
  red_mask = red1_mask | red2_mask;

  //Mat mask = getStructuringElement(MORPH_RECT,Size(5,5),Point(1,1));
  GaussianBlur(blue_mask, blue_mask, cv::Size(5,5),0);
  dilate(blue_mask,blue_mask,Mat());
  erode(blue_mask,blue_mask,Mat());  // smoothing

  GaussianBlur(green_mask, green_mask, cv::Size(5,5),0);
  dilate(green_mask,green_mask,Mat());
  erode(green_mask,green_mask,Mat());  // smoothing

  GaussianBlur(red_mask, red_mask, cv::Size(5,5),0);
  dilate(red_mask,red_mask,Mat());
  erode(red_mask,red_mask,Mat());  // smoothing


  Mat blue,green,red;
  buffer.copyTo(red,red_mask);
  buffer.copyTo(green,green_mask);
  buffer.copyTo(blue,blue_mask);

  Mat red_gray,green_gray,blue_gray;
  cvtColor(red, red_gray, CV_RGB2GRAY);
  cvtColor(green, green_gray, CV_RGB2GRAY);
  cvtColor(blue, blue_gray, CV_RGB2GRAY);

  vector<Vec3f> circles; //assign a memory to save the result of circle detection
  HoughCircles(gray,circles,HOUGH_GRADIENT, 1, 20, 50, 35, 0,0); //proceed circle detection
  vector<Vec3f> red_circles,green_circles,blue_circles;
  HoughCircles(red_mask,red_circles,HOUGH_GRADIENT, 1, 100, 50, 20, 0,0); //proceed circle detection
  HoughCircles(green_mask,green_circles,HOUGH_GRADIENT, 1, 100, 50, 20, 0,0); //proceed circle detection
  HoughCircles(blue_mask,blue_circles,HOUGH_GRADIENT, 1, 70, 100, 20, 0,0); //proceed circle detection

  Vec3f params; //assign a memory to save the information of circles
  float cx,cy,r;
  cout<<"circles.size="<<circles.size()<<endl;  //print the number of circles detected
  cout<<"red_circles.size="<<red_circles.size()<<endl;  //print the number of circles detected
  cout<<"green_circles.size="<<green_circles.size()<<endl;  //print the number of circles detected
  cout<<"blue_circles.size="<<blue_circles.size()<<endl;  //print the number of circles detected

  core_msgs::ball_info msg;  //create a message for ball position

  /*
  msg.size =circles.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  msg.img_x.resize(circles.size());  //adjust the size of array
  msg.img_y.resize(circles.size());  //adjust the size of array

  core_msgs::ball_position msg_r;  //create a message for ball positions
  msg_r.size =red_circles.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  msg_r.img_x.resize(red_circles.size());  //adjust the size of array
  msg_r.img_y.resize(red_circles.size());  //adjust the size of array

  core_msgs::ball_position msg_g;  //create a message for ball positions
  msg_g.size =green_circles.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  msg_g.img_x.resize(green_circles.size());  //adjust the size of array
  msg_g.img_y.resize(green_circles.size());  //adjust the size of array

  core_msgs::ball_position msg_b;  //create a message for ball positions
  msg_b.size =blue_circles.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
  msg_b.img_x.resize(blue_circles.size());  //adjust the size of array
  msg_b.img_y.resize(blue_circles.size());  //adjust the size of array

  visualization_msgs::Marker ball_list;  //declare marker
  ball_list.header.frame_id = "/camera_link";  //set the frame
  ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
  ball_list.ns = "balls";   //name of markers
  ball_list.action = visualization_msgs::Marker::ADD;
  ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
  ball_list.pose.position.y=0;
  ball_list.pose.position.z=0;
  ball_list.pose.orientation.x=0;
  ball_list.pose.orientation.y=0;
  ball_list.pose.orientation.z=0;
  ball_list.pose.orientation.w=1.0;

  ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
  ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker

  double radius = 0.10;
  ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
  ball_list.scale.y=radius;
  ball_list.scale.z=radius;

  for(int k=0;k<circles.size();k++){
    params = circles[k];  //the information of k-th circle
    cx=cvRound(params[0]);  //x position of k-th circle
    cy=cvRound(params[1]);  //y position
    r=cvRound(params[2]); //radius
       // 원 출력을 위한 원 중심 생성
    Point center(cx,cy);  //declare a Point
    circle(gray,center,r,Scalar(0,0,0),10); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth

    // cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
    // cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);

    msg.img_x[k]=cx;  //input the x position of the ball to the message
    msg.img_y[k]=cy;

    geometry_msgs::Point p;
    p.x = cx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
    p.y = cy;
    p.z = 0.1;
    ball_list.points.push_back(p);

    std_msgs::ColorRGBA c;
    c.r = 0.0;  //set the color of the balls. You can set it respectively.
    c.g = 1.0;
    c.b = 0.0;
    c.a = 1.0;
    ball_list.colors.push_back(c);
  }
  */

  int ball_number = 0;

  for(int bi=0;bi<blue_circles.size();bi++){
    params = blue_circles[bi];  //the information of k-th circle
    cx=cvRound(params[0]);  //x position of k-th circle
    cy=cvRound(params[1]);  //y position
    r=cvRound(params[2]); //radius
    // 원 출력을 위한 원 중심 생성
    Point blue_center(cx,cy);  //declare a Point
    circle(result,blue_center,r,Scalar(0,0,255),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth

    vector<float> ball_pos_b;
    ball_pos_b = pixel2point(cx, cy, r);

    float isx = ball_pos_b[0];
    float isy = ball_pos_b[1];
    float isz = ball_pos_b[2];

    string sx = to_string(isx);
    string sy = to_string(isy);
    string sz = to_string(isz);

    string text_b;
    text_b = "x: " + sx +", y: " + sy + ", z: " + sz + ", r: " + to_string(r);
    putText(result, text_b, blue_center, 1, 1, Scalar(0,0,255), 2);

    msg.img_x.push_back(ball_pos_b[0]);
    msg.img_y.push_back(ball_pos_b[2]);
    msg.color.push_back(1);
    ball_number++;
  }

  for(int ri=0;ri<red_circles.size();ri++){
    params = red_circles[ri];  //the information of k-th circle
    cx=cvRound(params[0]);  //x position of k-th circle
    cy=cvRound(params[1]);  //y position
    r=cvRound(params[2]); //radius
    // 원 출력을 위한 원 중심 생성
    Point red_center(cx,cy);  //declare a Point
    circle(result,red_center,r,Scalar(255,0,0),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth

    vector<float> ball_pos_r;
    ball_pos_r = pixel2point(cx, cy, r);

    float isx = ball_pos_r[0];
    float isy = ball_pos_r[1];
    float isz = ball_pos_r[2];

    string sx = to_string(isx);
    string sy = to_string(isy);
    string sz = to_string(isz);

    string text_r;
    text_r = "x: " + sx +", y: " + sy + ", z: " + sz + ", r: " + to_string(r);
    putText(result, text_r, red_center, 1, 1, Scalar(255,0,0), 2);

    msg.img_x.push_back(ball_pos_r[0]);
    msg.img_y.push_back(ball_pos_r[2]);
    msg.color.push_back(2);
    ball_number++;
  }

  for(int gi=0;gi<green_circles.size();gi++){
    params = green_circles[gi];  //the information of k-th circle
    cx=cvRound(params[0]);  //x position of k-th circle
    cy=cvRound(params[1]);  //y position
    r=cvRound(params[2]); //radius
    // 원 출력을 위한 원 중심 생성
    Point green_center(cx,cy);  //declare a Point
    circle(result,green_center,r, Scalar(0,255,0),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth

    vector<float> ball_pos_g;
    ball_pos_g = pixel2point(cx, cy, r);

    float isx = ball_pos_g[0];
    float isy = ball_pos_g[1];
    float isz = ball_pos_g[2];

    string sx = to_string(isx);
    string sy = to_string(isy);
    string sz = to_string(isz);

    string text_g;
    text_g = "x: " + sx +", y: " + sy + ", z: " + sz + ", r: " + to_string(r);
    putText(result, text_g, green_center, 1, 1, Scalar(0,255,0), 2);

    msg.img_x.push_back(ball_pos_g[0]);
    msg.img_y.push_back(ball_pos_g[2]);
    msg.color.push_back(3);
    ball_number++;
  }

  msg.number = ball_number;

  cv::imshow("view", result);  //show the image with a window
  cv::imshow("gray", gray);
  cv::imshow("blue",blue_mask);
  cv::imshow("green",green_mask);
  cv::imshow("red",red_mask);

  cv::waitKey(1);
  pub.publish(msg);  //publish a message
  /*
  pub_r.publish(msg_r);  //publish a message
  pub_g.publish(msg_g);  //publish a message
  pub_b.publish(msg_b);  //publish a message
  pub_markers.publish(ball_list);  //publish a marker message
  */
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  ball_detect(); //proceed ball detection
}

/*
void depthCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv_bridge::CvImageConstPtr depth_img_cv;
    depth_img_cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img_cv->image.convertTo(buffer_depth, CV_32F, 0.001);
    Mat disp_depth;
    double minVal, maxVal;
    minMaxLoc(buffer_depth, &minVal, &maxVal);
    buffer_depth.convertTo(disp_depth, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    double minVd, maxVd;
    minMaxLoc(depth_img_cv->image, &minVd, &maxVd);

    double minVb, maxVb;
    minMaxLoc(buffer_depth, &minVb, &maxVb);

    for(int i = 0; i<depth_img_cv->image.cols; i++){
      for(int j = 0; j<depth_img_cv->image.rows; j++){
        float test = depth_img_cv->image.at<float>(j, i);
        if(test > 1){
          cout<<"Well, here : "<<i<<", "<<j<<", the value is : "<<test<<endl;
          circle(disp_depth,Point(i,j),1, Scalar(255,255,0),-1);
        }
      }
    }

    Scalar mean_value;
    mean_value = mean(buffer_depth, blue_mask);

    float mean_val = sum(mean_value)[0];

    ROS_INFO("%f", mean_val);

    cout<<"raw min, max "<<minVd<<", "<<maxVd<<endl;
    cout<<"buffer min, max "<<minVb<<", "<<maxVb<<endl;


    imshow("depth", disp_depth);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

*/

int main(int argc, char **argv){
  ros::init(argc, argv, "ball_detect_node"); //init ros nodd
  ros::NodeHandle nh; //create node handler
  image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback); //create subscriber
  //image_transport::Subscriber sub_depth = it.subscribe("/camera/depth/image_raw", 1, depthCallback);

  pub = nh.advertise<core_msgs::ball_info>("/position", 100); //setting publisher

  /*
  pub_r = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
  pub_g = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
  pub_b = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher

  pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);
  */

  ros::spin(); //spin.
  return 0;
}
