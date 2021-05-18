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

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int MAP_WIDTH = 600;
int MAP_HEIGHT = 400;
int MAP_CENTER = 50;
int ZONE_SIZE = 5;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int nBalls;
float ballDist[20];
float ballAngle[20];         // Ball position info

float goalDist, goalAngle;   // Goal position info

float X, Y, O;               // Odometry info

class Zones
{
  class Zone
  {
  public:
    int nPoints;
    float cenRow; //(row,col)
    float cenCol;
    bool harvested = false;
    Zone(int _row, int _col);
    ~Zone();
    bool insideZone(int r, int c);
    void add(int r, int c);
  };

public:
  vector<Zone> zoneList;
  Zones();
  ~Zones();
  int size();
  void addZone(int r, int c);
  void deleteZone(int i);
};

Zones::Zones(){}
Zones::~Zones(){}

int Zones::size()
{
  return zoneList.size();
}

void Zones::addZone(int r, int c)
{
  int n = zoneList.size();
  for (int i=0;i<n;i++){
    if (zoneList[i].insideZone(r,c)){
      zoneList[i].add(r,c);
      return;
    }
  }
  zoneList.push_back(Zone(r,c));
}

Zones::Zone::Zone(int r, int c):nPoints(1),cenRow(r),cenCol(c),harvested(false){}
Zones::Zone::~Zone(){}

bool Zones::Zone::insideZone(int r, int c)
{
  return (r >= cenRow-ZONE_SIZE && r <= cenRow+ZONE_SIZE && c >= cenCol-ZONE_SIZE && c <= cenCol+ZONE_SIZE);
}

void Zones::Zone::add(int r, int c)
{
  cenRow = (nPoints*cenRow + r)/(nPoints+1);
  cenCol = (nPoints*cenCol + c)/(nPoints+1);
  ++nPoints;
}

Mat mapBall = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_16U);
Mat mapGoal = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_16U);
Mat mapPillar = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_16U);
Mat MAP = cv::Mat::zeros(MAP_HEIGHT,MAP_WIDTH, CV_8UC3);     // final map

Zones ballZones;
Zones pillarZones;

#define RAD2DEG(x) ((x)*180./M_PI)


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
    goalAngle = pos->angle;
    goalDist = pos->dist;
}

void odometry_Callback(const geometry_msgs::Vector3 odometry){
    X = odometry.x;
    Y = odometry.y;
    O = odometry.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_show_node");
    ros::NodeHandle n;

    ros::Subscriber subOdo = n.subscribe<geometry_msgs::Vector3>("/odometry", 1000, odometry_Callback);
    ros::Subscriber subBall = n.subscribe<core_msgs::goal_position>("/goal_position", 1000, goalPos_Callback);
    ros::Subscriber subGoal = n.subscribe<core_msgs::ball_position>("/ball_position", 1000, ballPos_Callback);
    line(MAP, Point(50, 50), Point(550, 50), Scalar(255,255,255), 1);
    line(MAP, Point(50, 50), Point(50, 250), Scalar(255,255,255), 1);
    line(MAP, Point(550, 50), Point(550, 350), Scalar(255,255,255), 1);
    line(MAP, Point(50, 350), Point(550, 350), Scalar(255,255,255), 1);
    while(ros::ok){
      circle(MAP, Point(50+int(round(X)), 350-int(round(Y))), 3, cv::Scalar(255,0,0), -1);
      for (int i=0; i<nBalls; i++){ //ball_dist[i], ball_angle[i]
        int ball_x = 50 + (int)round(X + ballDist[i]*cos(ballAngle[i]+O)*100);
        int ball_y = 350 - (int)round(Y + ballDist[i]*sin(ballAngle[i]+O)*100);
        if (ball_x>50 && ball_x<550 && ball_y>50 && ball_y<350){
          circle(MAP, Point(ball_x, ball_y), 3, cv::Scalar(0,0,255), -1);
          mapBall.at<ushort>(ball_y,ball_x)++;
          ballZones.addZone(ball_y,ball_x);
        }
      }

      int bzSize = ballZones.size();
      Mat target;
      for (int i=0; i<bzSize; i++){
        target = mapBall(Rect(Point(ballZones.zoneList[i].cenCol-ZONE_SIZE,ballZones.zoneList[i].cenRow-ZONE_SIZE),Point(ballZones.zoneList[i].cenCol+ZONE_SIZE,ballZones.zoneList[i].cenRow+ZONE_SIZE)));
        cout << i << "-th zone : " << target.size() << endl;

      }
      // if (ball_x>50 && ball_x<550 && ball_y>50 && ball_y<350){
      //   mapBall.at<ushort>(ball_y, ball_x) += 1;
      //   circle(MAP, Point(ball_x, ball_y),2,cv::Scalar(0,0,255), -1);
      // }

      imshow("map", MAP);
      waitKey(10);
      // if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
      //     return 0;
      // }
      ros::spinOnce();
    }
    return 0;
}
