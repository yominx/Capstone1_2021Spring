#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "std_msgs/Int8.h"
#include "core_msgs/ball_info.h"
#include "core_msgs/to_astar.h"

using namespace cv;
using namespace std;

ros::Publisher pub;

float view_point_x[5] = {4.0,3.5,-1.0,4.5,7.5};
float view_point_y[5] = {0.5,2.5,-1.0,1.6,2.5};
float view_point_o[5] = {2*atan(1),-0.2,atan2f(-3,5),atan2f(-3,5), atan2f(-3,-5)};
int view_point = 0;

vector<float> red_x;
vector<float> red_y;
vector<float> blue_x;
vector<float> blue_y;

vector<float> red_x_rel;
vector<float> red_y_rel;
vector<float> blue_x_rel;
vector<float> blue_y_rel;

int mode = 0; //-1 : still in entrance zone  0 : search mode 1: harvest mode
int arrived = 1;
int to_goal = 0;
int to_VP = 1;
int mission = 3;

float obj_x, obj_y, obj_o;
float pos_x, pos_y, pos_o;

void toAstar(float x, float y, float o, vector<float> balls_x, vector<float> balls_y){
  core_msgs::to_astar msg;
  msg.destination.push_back(x);
  msg.destination.push_back(y);
  msg.destination.push_back(o);

  for(int i = 0; i<balls_x.size(); i++){
    msg.balls_x.push_back(balls_x[i]);
    msg.balls_y.push_back(balls_y[i]);
  }
  pub.publish(msg);
}

void ball_Update(){
  float temp_x;
  float temp_y;

  for(int i = 0; i<red_x_rel.size(); i++){
    temp_x = red_x_rel[i];
    temp_y = red_y_rel[i];

    for(int j = 0; j<red_x.size(); j++){
      if(sqrt(pow(temp_x - red_x[j],2)+pow(temp_x - red_x[j],2)) < 0.14){
        red_x[j] = temp_x;
        red_y[j] = temp_y;
      }
      else{
        red_x.push_back(temp_x);
        red_y.push_back(temp_y);
      }
    }
  }

  for(int i = 0; i<blue_x_rel.size(); i++){
    temp_x = blue_x_rel[i];
    temp_y = blue_y_rel[i];

    for(int j = 0; j<blue_x.size(); j++){
      if(sqrt(pow(temp_x - blue_x[j],2)+pow(temp_x - blue_x[j],2)) < 0.14){
        blue_x[j] = temp_x;
        blue_y[j] = temp_y;
      }
      else{
        blue_x.push_back(temp_x);
        blue_y.push_back(temp_y);
      }
    }
  }
  cout<<"Ball Updated"<<endl;
  to_VP = 1;
  view_point++;
}

void ball_Callback(const core_msgs::ball_info::ConstPtr& position)
{
  if(position != NULL){
    int count = position->number;

    blue_x_rel.clear();
    blue_y_rel.clear();
    red_x_rel.clear();
    red_y_rel.clear();

    for(int i = 0; i < count; i++)
    {
      int color = position->color[i];
      if(color == 1){
        blue_x_rel.push_back(position->img_x[i]);
        blue_y_rel.push_back(position->img_y[i]);
      }
      else if(color == 2){
        red_x_rel.push_back(position->img_x[i]);
        red_y_rel.push_back(position->img_y[i]);
      }
    }
  }
}

void motor_Callback(const std_msgs::Int8::ConstPtr& arrival)
{
  if(arrival != NULL){
    int arr = arrival -> data;
    if(arrived == 1 && arr == 0){
      arrived = 0;
      cout<<"On Move"<<endl;
    }
    if(arrived == 0 && arr == 1){
      if(mode == 0 && to_VP == 1){
        to_VP = 0;
        cout<<"View point "<<view_point+1<<" arrived"<<endl;
      }
      else if(mode == 1 && to_goal == 0){
        blue_x.pop_back();
        blue_y.pop_back();
        to_goal = 1;
        cout<<"Blue ball "<<4-mission<<" acquired"<<endl;
      }
      else if(mode == 1 && to_goal == 1){
        cout<<"Blue ball "<<4-mission<<" in the goal"<<endl;
        mission--;
        to_goal = 0;
      }
      arrived = 1;
    }
  }
  else{
    arrived = 0;
  }
}

void line_Callback(const std_msgs::Int8::ConstPtr& entrance)
{
  if(entrance != NULL){
    int ent = entrance -> data;
    if(mode == -1 && ent == 0){
      mode = 0;
      cout<<"Ball harvesting initiated"<<endl;
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ball_harvest_node");
  ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe<std_msgs::Int8>("/entrance", 1, line_Callback);
  ros::Subscriber sub1 = nh.subscribe<std_msgs::Int8>("/arrival", 1, motor_Callback);
  ros::Subscriber sub2 = nh.subscribe<core_msgs::ball_info>("/position", 1, ball_Callback);

  pub = nh.advertise<core_msgs::to_astar>("/to_astar", 1);

  int stabilize = 0;

  while(ros::ok){
    if(arrived == 1){
      if(view_point < 4 || (blue_x.size() == 0 && mission > 0)){
        mode = 0;
        cout<<"Search Mode"<<endl;
      }
      else if(mode == 0){
        mode = 1;
        cout<<"Harvest Mode"<<endl;
      }

      if(mode == -1){
        vector<float> nob_x;
        vector<float> nob_y;
        toAstar(-1.0, -1.0, 100.0, nob_x, nob_y);
        cout<<"Still in the Entrance Zone"<<endl;
      }
      else if(mode == 0){
        if(to_VP == 1){

          vector<float> nob_x;
          vector<float> nob_y;

          obj_x = view_point_x[view_point];
          obj_y = view_point_y[view_point];
          obj_o = view_point_o[view_point];

          for(int i = 0; i<red_x.size(); i++){
            if(sqrt(pow(obj_x - red_x[i],2)+pow(obj_y - red_y[i],2)) < 0.2){
              obj_x = -1.0;
              obj_y = -1.0;
              obj_o = 100.0;
            }
          }
          for(int i = 0; i<blue_x.size(); i++){
            if(sqrt(pow(obj_x - blue_x[i],2)+pow(obj_y - blue_y[i],2)) < 0.2){
              obj_x = -1.0;
              obj_y = -1.0;
              obj_o = 100.0;
            }
          }

          for(int i = 0; i<red_x.size(); i++){
            nob_x.push_back(red_x[i]);
            nob_y.push_back(red_y[i]);
          }
          for(int i = 0; i<blue_x.size(); i++){
            nob_x.push_back(blue_x[i]);
            nob_y.push_back(blue_y[i]);
          }

          toAstar(obj_x, obj_y, obj_o, nob_x, nob_y);
          cout<<"Destination : View point "<<view_point+1<<endl;
        }

        else{
          vector<float> nob_x;
          vector<float> nob_y;
          toAstar(-1.0, -1.0, 100.0, nob_x, nob_y);
          if(stabilize = 0){
            cv::waitKey(1000); //영상 안정화 한번 하고 다시 loop
            stabilize = 1;
          }
          else{
            ball_Update();
            stabilize = 0;
          }
        }
      }

      else if(mode == 1){
        if(to_goal = 0){
          vector<float> nob_x;
          vector<float> nob_y;

          obj_x = blue_x.back();
          obj_y = blue_y.back();
          obj_o = 0.0;

          for(int i = 0; i<red_x.size(); i++){
            nob_x.push_back(red_x[i]);
            nob_y.push_back(red_y[i]);
          }
          for(int i = 0; i<blue_x.size()-1; i++){
            nob_x.push_back(blue_x[i]);
            nob_y.push_back(blue_y[i]);
          }

          toAstar(obj_x, obj_y, obj_o, nob_x, nob_y);
          cout<<"Destination : Blue Ball "<<4 - mission<<endl;
        }
        else if(to_goal = 1){
          vector<float> nob_x;
          vector<float> nob_y;

          for(int i = 0; i<red_x.size(); i++){
            nob_x.push_back(red_x[i]);
            nob_y.push_back(red_y[i]);
          }
          for(int i = 0; i<blue_x.size(); i++){
            nob_x.push_back(blue_x[i]);
            nob_y.push_back(blue_y[i]);
          }

          toAstar(7.5, 1.5, 0.0, nob_x, nob_y);
          cout<<"Destination : Goal Post"<<endl;
        }
      }
    }

    ros::spinOnce();
  }

  return 0;
}
