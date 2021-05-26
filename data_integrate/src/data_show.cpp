#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <boost/thread.hpp>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/LaserScan.h"
#include "core_msgs/ball_position.h"
#include "core_msgs/goal_position.h"
#include "geometry_msgs/Vector3.h"
#include "core_msgs/multiarray.h"
#include "std_msgs/Int8MultiArray.h"

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#define VEHICLE 0
#define BALL 1
#define PILLAR 2
#define GOAL 3

int MAP_WIDTH = 600;
int MAP_HEIGHT = 400;
int MAP_CENTER = 50;
int ZONE_SIZE = 5;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int nBalls=0;
float ballDist[20];
float ballAngle[20];         // Ball position info

int nGoals=0;
float goalDist[3];
float goalAngle[3];   // Goal position info

int nPillars=0;
float pillarDist[10];
float pillarAngle[10];

float X, Y, O;               // Odometry info

Mat mapBall = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_32S);
Mat mapGoal = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_32S);
Mat mapPillar = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_32S);
Mat MAP = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_8UC3);     // final map

class Zones
{
  class Zone
  {
  public:
    int type;
    int nPoints;
    int cenRow; //(row,col)
    int cenCol;
    int cnt;
    bool reliable;
    int zoneSize;
    float threshold;
    Zone(int r, int c, int type);
    ~Zone();
    bool insideZone(int r, int c);
    void add(int r, int c, int type);
  };

public:
  vector<Zone> zoneList;
  Zones();
  ~Zones();
  int size();
  void addZone(int r, int c, int type);
  void removeZone(int i, int type);
};

Zones::Zones(){}

Zones::~Zones(){}

int Zones::size()
{
  return zoneList.size();
}

void Zones::addZone(int r, int c, int type)
{
  int n = zoneList.size();
  cout << "zoneList size = " << n << endl;
  for (int i=0;i<n;i++){
    if (zoneList[i].insideZone(r,c)){
      zoneList[i].add(r,c,type);
      return;
    }
  }
  zoneList.push_back(Zone(r,c,type));
}

void Zones::removeZone(int i, int type)
{
  Mat map;
  switch(type){
    case BALL:
      map = mapBall;
      break;
    case PILLAR:
      map = mapPillar;
      break;
    case GOAL:
      map = mapGoal;
  }
  Zone zone = zoneList[i];
  Rect roi = Rect(Point(zone.cenCol-zone.zoneSize,zone.cenRow-zone.zoneSize),Point(zone.cenCol+zone.zoneSize,zone.cenRow+zone.zoneSize));
  map(roi) = Scalar(0);
  circle(MAP, Point(zone.cenCol, zone.cenRow),2,Scalar(0,0,0), -1);
  zoneList.erase(zoneList.begin()+i);
  cout << "zone " << i << "-th removed" << endl;
}

Zones::Zone::Zone(int r, int c, int type):type(type),nPoints(1),cenRow(r),cenCol(c),cnt(1),reliable(false)
{
  switch(type){
    case BALL:
      zoneSize = 20;
      threshold = 0.3;
      break;
    case PILLAR:
      zoneSize = 20;
      threshold = 0.1;
      break;
    case GOAL:
      zoneSize = 50;
      threshold = 0.01;
  }
}
Zones::Zone::~Zone(){}

bool Zones::Zone::insideZone(int r, int c)
{
  return (r >= cenRow-zoneSize && r <= cenRow+zoneSize && c >= cenCol-zoneSize && c <= cenCol+zoneSize);
}

void Zones::Zone::add(int r, int c, int type)
{
  Mat map;
  switch(type){
    case BALL:
      map = mapBall;
      break;
    case PILLAR:
      map = mapPillar;
      break;
    case GOAL:
      map = mapGoal;
  }
  if (map.at<int>(r,c) > map.at<int>(cenRow,cenCol)){
    circle(MAP, Point(cenCol, cenRow),2,Scalar(0,0,0), -1);
    cenRow = r;
    cenCol = c;
  }
  ++nPoints;
}

Zones ballZones;
Zones pillarZones;
Zones goalZones;

void filtering(Zones& zones, int size, float* dist, float* angle, int type, core_msgs::multiarray& msg)
{
  Mat map;
  Scalar color;
  switch(type){
    case BALL:
      map = mapBall;
      color = Scalar(0,0,255);
      break;
    case PILLAR:
      map = mapPillar;
      color = Scalar(0,255,255);
      break;
    case GOAL:
      map = mapGoal;
      color = Scalar(0,255,0);
  }

  for (int i=0; i<size; i++){ //ball_dist[i], ball_angle[i]
    int x = 50 + (int)round(X + (dist[i]*cos(angle[i] + O)*100));
    int y = 350 - (int)round(Y + (dist[i]*sin(angle[i] + O)*100));
    if (!(x>50 && x<=550 && y>50 && y<350)){
      continue;
    }
    map.at<int>(y,x) += 1;
    zones.addZone(y,x,type);
  }

  int zSize = zones.zoneList.size();
  for (int i=0,j=0; i<zSize; i++, j++){
    zones.zoneList[j].cnt++;
    // cout << "(" << j << "-th zone) nPoints: " << zones.zoneList[j].nPoints << ", cnt: "<< zones.zoneList[j].cnt << endl;
    // cout << "(" << j << "-th zone) is reliable : " << zones.zoneList[j].reliable << endl;
    // cout << "(" << j << "-th zone) cnt*threshold = " << zones.zoneList[j].cnt << " * " << zones.zoneList[j].threshold << " = " << zones.zoneList[j].cnt * zones.zoneList[j].threshold <<endl;
    if ((zones.zoneList[j].cnt % 30) == 0){
      if (zones.zoneList[j].nPoints > zones.zoneList[j].cnt * zones.zoneList[j].threshold){
        zones.zoneList[j].reliable = true;
        // cout << "(" << j << "-th zone) is reliable" << endl;
      }
      else {
        zones.removeZone(j, type);
        j--;
      }
    }
    if (zones.zoneList[j].reliable){
      msg.data.push_back(type);
      msg.data.push_back(zones.zoneList[j].cenCol);
      msg.data.push_back(zones.zoneList[j].cenRow);
      circle(MAP, Point(zones.zoneList[j].cenCol, zones.zoneList[j].cenRow),5,color, -1);
      continue;
    }
  }
}

void ballPos_Callback(const core_msgs::ball_position::ConstPtr& pos)
{
    nBalls = pos->size;
    for(int i = 0; i < nBalls; i++)
    {
        ballAngle[i] = pos->angle[i];
        ballDist[i] = pos->dist[i];
    }
}
void goalPos_Callback(const core_msgs::goal_position::ConstPtr& pos)
{
    nGoals = 1;
    goalAngle[0] = pos->angle;
    goalDist[0] = pos->dist;
}

void odometry_Callback(const geometry_msgs::Vector3 odometry){
    //cout << "(X,Y,O)= (" << X << ", " << Y << ", " << O << ")"  << endl;
    X = odometry.x;
    Y = odometry.y;
    O = odometry.z;
}

void pillarPos_Callback(const std_msgs::Int8MultiArray pos)
{
    nPillars = pos.data.size()/2;
    for (int i=0; i<nPillars; i++){
      pillarDist[i] = pos.data[2*i];
      pillarAngle[i] = pos.data[2*i+1];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<core_msgs::multiarray>("/position", 1000); //odometry, 즉 robot의 위치를 Vector3로 발행한다.
    ros::Subscriber subOdo = n.subscribe<geometry_msgs::Vector3>("/robot_pos", 1000, odometry_Callback);
    ros::Subscriber subBall = n.subscribe<core_msgs::goal_position>("/goal_position", 1000, goalPos_Callback);
    ros::Subscriber subGoal = n.subscribe<core_msgs::ball_position>("/ball_position", 1000, ballPos_Callback);
    ros::Subscriber subPillar = n.subscribe<std_msgs::Int8MultiArray>("/obs_pos", 1000, pillarPos_Callback);
    ros::Rate loop_rate(10);
    line(MAP, Point(50, 50), Point(550, 50), Scalar(255,255,255), 1);
    line(MAP, Point(50, 50), Point(50, 250), Scalar(255,255,255), 1);
    line(MAP, Point(550, 50), Point(550, 350), Scalar(255,255,255), 1);
    line(MAP, Point(50, 350), Point(550, 350), Scalar(255,255,255), 1);
    // X=60.;
    // Y=150.;
    // O=0.;

    while(ros::ok){
      core_msgs::multiarray msg;
      filtering(ballZones, nBalls, ballDist, ballAngle, BALL, msg);
      filtering(pillarZones, nPillars, pillarDist, pillarAngle, PILLAR, msg);
      filtering(goalZones, nGoals, goalDist, goalAngle, GOAL, msg);
      circle(MAP, Point(50+int(round(X)), 350-int(round(Y))), 5, cv::Scalar(255,0,0), -1);
      msg.data.push_back(VEHICLE);
      msg.data.push_back(50+int(round(X)));
      msg.data.push_back(350-int(round(Y)));
      msg.cols = 1+nBalls+nGoals+nPillars;
      pub.publish(msg);
      imshow("map", MAP);
      waitKey(10);
      loop_rate.sleep();
      ros::spinOnce();
    }
    return 0;
}
