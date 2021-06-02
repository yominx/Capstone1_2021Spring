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
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
// #define RAW
// #define DEBUG

#define VEHICLE 0
#define BALL 1
#define PILLAR 2
#define GOAL 3

int MAP_WIDTH = 600;
int MAP_HEIGHT = 400;
float DLC = 0.27; //distance between camara and lidar
float DLB = 0.50; //distance between lidar and ball

int nData = 0;

int nBalls=0;
int remainBalls = 5;
float ballDist[20];
float ballAngle[20];         // Ball position info

int nGoals=0;
float goalDist[3];
float goalAngle[3];   // Goal position info

int nPillars=0;
float pillarDist[10];
float pillarAngle[10];

float X, Y, O;               // Odometry info
bool odometryCalled = false;

float angular_vel=0;

Mat mapBall = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_32S);
Mat mapGoal = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_32S);
Mat mapPillar = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_32S);
Mat MAP = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_8UC3);     // final map

#ifdef RAW
Mat MAPRAW = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_8UC3);
#endif

#ifdef DEBUG
Mat mapBallDebug;
Mat mapGoalDebug;
Mat mapPillarDebug;
#endif

vector<int> reliableList;
core_msgs::multiarray msg;


int delivery_mode=0;
void delivery_mode_Callback(const std_msgs::Int8::ConstPtr& delivery){
  delivery_mode=delivery->data;
}


float ALPHA = 0.5;
class Zones
{
public:
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
  int max;
  Zones(int goal);
  ~Zones();
  int size();
  void addZone(int r, int c, int type);
  void removeZone(int r, int c, int type);
  void removeZone(int i, int type);
};

Zones::Zones(int goal):max(goal){}

Zones::~Zones(){}

int Zones::size()
{
  return zoneList.size();
}

void Zones::addZone(int r, int c, int type)
{
  int n = zoneList.size();
  for (int i=0;i<n;i++){
    if (zoneList[i].insideZone(r,c)){
      zoneList[i].add(r,c,type);
      return;
    }
  }
  zoneList.push_back(Zone(r,c,type));
}

void Zones::removeZone(int r, int c, int type)
{
  int n = zoneList.size();
  float minDistsq = 9999999., tmpDistsq;
  int minIdx;
  for (int i=0;i<n;i++){
    if (zoneList[i].reliable && zoneList[i].type == type ) {
      tmpDistsq = pow(r-zoneList[i].cenRow,2) + pow(c-zoneList[i].cenCol,2);
      if (tmpDistsq < minDistsq){
        minDistsq = tmpDistsq;
        minIdx = i;
      }
      cout << "Remove \n\ttype:" << zoneList[i].type <<
      "(x, y): " << zoneList[i].cenCol<< ", " << zoneList[i].cenRow << endl;
    }
  }
  removeZone(minIdx,type);
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
  if(i >= zoneList.size()){
    cerr << "INVALID INDEX" << i << "is larger than " << zoneList.size() << endl;
  }
  Zone zone = zoneList[i];
  int x = zone.cenCol, y = zone.cenRow, delta=zone.zoneSize;
  cout << "RECTANGULAR: " << x << " and " << y << " DIFF " << delta << endl;
  Rect roi = Rect(Point(x-delta, y-delta),Point(x+delta, y+delta));
  map(roi) = Scalar(0);
  circle(MAP, Point(zone.cenCol, zone.cenRow),2,Scalar(0,0,0), -1);
  zoneList.erase(zoneList.begin()+i);
}

Zones::Zone::Zone(int r, int c, int type):type(type),nPoints(1),cenRow(r),cenCol(c),cnt(1),reliable(false)
{
  switch(type){
    case BALL:
      zoneSize = 35;
      threshold = 0.3;
      break;
    case PILLAR:
      zoneSize = 50;
      threshold = 0.65;
      break;
    case GOAL:
      zoneSize = 50;
      threshold = 0.5;
  }
}
Zones::Zone::~Zone(){}

bool Zones::Zone::insideZone(int r, int c)
{
  int ball_dist_sq = pow(cenRow-r, 2) + pow(cenCol-c,2);
  // cout << "DISTANCE IS "<<  ball_dist_sq << endl;
  return (ball_dist_sq  <= pow(zoneSize,2) );
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
  map.at<int>(r,c) += 1;
  // if (map.at<int>(r,c) > map.at<int>(cenRow,cenCol)){
    circle(MAP, Point(cenCol, cenRow),2,Scalar(0,0,0), -1);
    cenRow = (cenRow == 0) ? r : ((1-ALPHA) * cenRow + ALPHA*r);
    cenCol = (cenCol == 0) ? c : ((1-ALPHA) * cenCol + ALPHA*c);
  // }
  ++nPoints;
}

Zones ballZones(5);
Zones pillarZones(3);
Zones goalZones(1);

void sort(vector<int>& reliableList, const Zones& zones)
{
  float keyVal, targetVal;
  int j;
  for (int i=1; i<reliableList.size(); i++){
    keyVal = zones.zoneList[reliableList[i]].nPoints / zones.zoneList[reliableList[i]].cnt;
    j=i-1;
    targetVal = zones.zoneList[reliableList[j]].nPoints / zones.zoneList[reliableList[j]].cnt;
    while (j >= 0 && targetVal < keyVal){
      reliableList[j+1] = reliableList[j];
      j -= 1;
      targetVal = zones.zoneList[reliableList[j]].nPoints / zones.zoneList[reliableList[j]].cnt;
    }
    reliableList[j] = reliableList[i];
  }
}

void filtering(Zones& zones, int size, float* dist, float* angle, int type, core_msgs::multiarray& msg)
{
  Mat map;
  Scalar color;
  switch(type){
    case BALL:
      color = Scalar(0,0,255);
      break;
    case PILLAR:
      color = Scalar(0,255,255);
      break;
    case GOAL:
      color = Scalar(0,255,0);
  }
  for (int i=0; i<size; i++){ //ball_dist[i], ball_angle[i]
    int x = (type==PILLAR) ? 50+(int)round(X +(dist[i]*cos(angle[i]+O)*100)) : 50+(int)round(X+(DLC*cos(O)*100)+(dist[i]*cos(angle[i]+O)*100));
    int y = (type==PILLAR) ? 350-(int)round(Y+(dist[i]*sin(angle[i]+O)*100)) : 350-(int)round(Y+(DLC*sin(O)*100)+(dist[i]*sin(angle[i]+O)*100));
    if (!(x>50 && x<=550 && y>50 && y<350)){
      continue;
    }

    if (fabs(angular_vel) < 0.5 || (delivery_mode!=1)) zones.addZone(y,x,type);
  }

  int zSize = zones.zoneList.size();
  reliableList.clear();
  for (int i=0,j=0; i<zSize; i++, j++){
    zones.zoneList[j].cnt++;
    // cout << "(" << j << "-th zone) nPoints: " << zones.zoneList[j].nPoints << ", cnt: "<< zones.zoneList[j].cnt << endl;
    // cout << "(" << j << "-th zone) is reliable : " << zones.zoneList[j].reliable << endl;
    // cout << "(" << j << "-th zone) cnt*threshold = " << zones.zoneList[j].cnt << " * " << zones.zoneList[j].threshold << " = " << zones.zoneList[j].cnt * zones.zoneList[j].threshold <<endl;
    if ((zones.zoneList[j].cnt % 10) == 0 && zones.zoneList[j].cnt < 100){
      if (zones.zoneList[j].nPoints > zones.zoneList[j].cnt * zones.zoneList[j].threshold){
        zones.zoneList[j].reliable = true;
        // reliableList.push_back(j);
        cout << "(" << j << "-th zone) is reliable: " << zones.zoneList[j].cenCol << ", " << zones.zoneList[j].cenRow << endl;
      }
      else {
        zones.removeZone(j, type);
        j--;
      }
    }
  }

  int nSize = zones.zoneList.size();
  for (int i=0; i<nSize; i++)
    if (zones.zoneList[i].reliable) reliableList.push_back(i);

  if (reliableList.size() > zones.max){
    sort(reliableList, zones); // sorting by the value of (nPoints/cnt)
  }

  int repeat = (reliableList.size()>zones.max) ? zones.max : reliableList.size();

  for (int i=0; i<repeat; i++){
    msg.data.push_back(type);
    msg.data.push_back(zones.zoneList[reliableList[i]].cenCol);
    msg.data.push_back(zones.zoneList[reliableList[i]].cenRow);
    nData += 1;
    circle(MAP, Point(zones.zoneList[reliableList[i]].cenCol, zones.zoneList[reliableList[i]].cenRow),2,color, -1);
  }
}

#ifdef DEBUG
void showColoredMap(int type)
{
  // Mat map;
  // Mat map_debug;
  // double minVal, maxVal;
  // Point minLoc, maxLoc;
  // Point matchLoc;
  switch(type){
    case BALL:
      mapBall.copyTo(mapBallDebug);
      // minMaxLoc(mapBall, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
      mapBallDebug.convertTo(mapBallDebug,CV_8UC3);
      imshow("colorMap", mapBallDebug);
      break;
    // case PILLAR:
    //   mapPillar.copyTo(mapPillarDebug);
    //   // minMaxLoc(mapPillar, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    //   break;
    // case GOAL:
    //   mapGoal.copyTo(map);
    //   color = Scalar(0,255,0);
    //   minMaxLoc(mapGoal, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
  }
}
#endif

#ifdef RAW
void drawRawMap(int type, int n, float* dist, float* angle)
{
  Scalar color;
  int x, y;
  switch(type){
    case BALL:
      color = Scalar(0,0,255);
      break;
    case PILLAR:
      color = Scalar(0,255,255);
      break;
    case GOAL:
      color = Scalar(0,255,0);
      break;
    case VEHICLE:
      color = Scalar(255,0,0);
  }
  for (int i=0; i<n; i++){
    int x, y;
    switch(type){
      case BALL:
        x = 50+(int)round(X+(DLC*cos(O)*100)+(dist[i]*cos(angle[i]+O)*100));
        y = 350-(int)round(Y+(DLC*sin(O)*100)+(dist[i]*sin(angle[i]+O)*100));
        break;
      case PILLAR:
        x = 50+(int)round(X +(dist[i]*cos(angle[i]+O)*100));
        y = 350-(int)round(Y+(dist[i]*sin(angle[i]+O)*100));
        break;
      case GOAL:
        x = 550;
        y = 200;
        break;
      case VEHICLE:
        x = X;
        y = Y;
    }
    circle(MAPRAW, Point(x,y), 2, color, -1);
  }
}
#endif

void ballPos_Callback(const core_msgs::ball_position::ConstPtr& pos)
{
  nBalls = pos->size;
  if (nBalls > 20) nBalls = 20;
  for(int i = 0; i < nBalls; i++)
  {
    ballAngle[i] = pos->angle[i];
    ballDist[i] = pos->dist[i];
  }
}
// void goalPos_Callback(const core_msgs::goal_position::ConstPtr& pos)
// {
//   nGoals = 1;
//   goalAngle[0] = (float)pos->angle;
//   goalDist[0] = pos->dist;
// }

void odometry_Callback(const geometry_msgs::Vector3 odometry){
  X = odometry.x;
  Y = odometry.y;
  O = odometry.z;
  odometryCalled = true;
}

void pillarPos_Callback(const std_msgs::Float32MultiArray pos)
{
  nPillars = pos.data.size()/2;
  if (nPillars > 10) nPillars=10;
  for (int i=0; i<nPillars; i++){
    pillarDist[i] = pos.data[2*i];
    pillarAngle[i] = pos.data[2*i+1];
  }
}



void goalNum_Callback(const std_msgs::Int8 msg)
{
    int tmp = 5 - msg.data;
    if (remainBalls != tmp){
      int xBall = 50 +(int)round(X + DLB*cos(O)*100);
      int yBall = 350-(int)round(Y + DLB*sin(O)*100);
      ballZones.removeZone(yBall,xBall,BALL);
      remainBalls--;
    }
}


void control_input_Callback(const geometry_msgs::Twist::ConstPtr& targetVel){
  angular_vel = targetVel->angular.z;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<core_msgs::multiarray>("/position", 1000); //odometry, 즉 robot의 위치를 Vector3로 발행한다.
    ros::Subscriber commandVel = n.subscribe<geometry_msgs::Twist>("/command_vel", 10, control_input_Callback);
    ros::Subscriber subOdo = n.subscribe<geometry_msgs::Vector3>("/robot_pos", 1000, odometry_Callback);
    // ros::Subscriber subBall = n.subscribe<core_msgs::goal_position>("/goal_position", 1000, goalPos_Callback);
    ros::Subscriber subGoal = n.subscribe<core_msgs::ball_position>("/ball_position", 1000, ballPos_Callback);
    ros::Subscriber subPillar = n.subscribe<std_msgs::Float32MultiArray>("/obs_pos", 1000, pillarPos_Callback);
    ros::Subscriber subGoalNum = n.subscribe<std_msgs::Int8>("/ball_number", 10, goalNum_Callback);
    ros::Subscriber delivery = n.subscribe<std_msgs::Int8>("/ball_delivery", 10, delivery_mode_Callback);


    ros::Rate loop_rate(10);
    line(MAP, Point(50, 50), Point(550, 50), Scalar(255,255,255), 1);
    line(MAP, Point(50, 50), Point(50, 250), Scalar(255,255,255), 1);
    line(MAP, Point(550, 50), Point(550, 350), Scalar(255,255,255), 1);
    line(MAP, Point(50, 350), Point(550, 350), Scalar(255,255,255), 1);
    // X=60.;
    // Y=150.;
    // O=0.;

    while(ros::ok){
      if (odometryCalled){
        msg.data.clear();
        msg.cols = 0;
        nData = 0;
        int i = 0;
        int k =0;
#ifdef RAW
        drawRawMap(BALL, nBalls, ballDist, ballAngle);
        drawRawMap(PILLAR, nPillars, pillarDist, pillarAngle);
        imshow("raw map", MAPRAW);
        waitKey(10);
#endif
        filtering(ballZones, nBalls, ballDist, ballAngle, BALL, msg);
        filtering(pillarZones, nPillars, pillarDist, pillarAngle, PILLAR, msg);
        //filtering(goalZones, nGoals, goalDist, goalAngle, GOAL, msg);
        circle(MAP, Point(50+int(round(X)), 350-int(round(Y))), 2, cv::Scalar(255,0,0), -1);
        msg.data.push_back(VEHICLE);
        msg.data.push_back(50+int(round(X)));
        msg.data.push_back(350-int(round(Y)));
        nData += 1;
#ifdef DEBUG
        showColoredMap(BALL);
#endif
        //set the goal position
        msg.data.push_back(GOAL);
        msg.data.push_back(550);
        msg.data.push_back(200);
        nData += 1;
        msg.cols = nData;
        pub.publish(msg);
        imshow("map", MAP);
        // destroyAllWindows();
        waitKey(1);
      }
      loop_rate.sleep();
      ros::spinOnce();
    }
    return 0;
}
