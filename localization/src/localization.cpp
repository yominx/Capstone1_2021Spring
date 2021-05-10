#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "core_msgs/ball_position.h"

#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;



float MAP_CX = 600;
float MAP_CY = 600;
float MAP_RESOL = 0.01;
int MAP_WIDTH = 1200;
int MAP_HEIGHT = 1200;
int MAP_CENTER = 50;


geometry_msgs::Vector3 odometry;
ros::Publisher pub;



int lidar_size;//lidar callback에서 사용되는 lidar point 개수
float lidar_degree[400]; //넉넉하게 400ㅇ로 한듯?
float lidar_distance[400];

int init_odom = 1; // Entrance node도 켰으면, 0으로 초기화!!

float control_x;
float control_y;
float control_o;


struct pos{
  float x=-5;
  float y=70;
  float o=0.1;
};
struct pos robot_pos;

float pos_x;
float pos_y;
float pos_o;


boost::mutex map_mutex;

float pi=2*atanf(1);
#define RAD2DEG(x) ((x)*180./M_PI)

// ball pos instances
int nBalls;
float ball_dist[20];
float ball_angle[20];


bool check_point_range(int cx, int cy) //어떤 input 위치가 MAP 안에 있는지 검사
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}

float vectorMean(std::vector<float> V){
    float total = 0;
    for (int i=0; i<V.size(); i++){
        total += V[i];
    }
    return (total /(float)V.size());
}




vector<float> lineAnalysis(Vec4i l){ //들어온 line detection으로부터 line의 slope 등 수학적 특징 추출
//Vec4i는 어떤 선분의 양 끝점의 x,y 좌표를 포함하는 int형 4개 x1, y1, x2, y2 로 이루어진 vector
  vector<float> line_info;
  float slope;
  float perp;
  float perp_x;
  float perp_y;
  float length;

  if(l[2]-l[0] == 0){ //선분이 수직일 때(0으로 나눌 수 없으니 따로 처리)
    slope = 2*atan(1);
    perp = (l[2]+l[0]-MAP_WIDTH)/2.0; //맵 중앙을 원점으로 했을 때 선분의 x좌표
    perp_x = perp; //
    perp_y = 0; //
  }
  else{ //맵 중앙 좌표계 기준 y=ax+b이라 하면
    slope = (l[3]-l[1])/(float)(l[2]-l[0]); //slope=a
    perp_x = (slope*(l[0]-MAP_WIDTH/2)-(l[1]-MAP_HEIGHT/2))*slope/(slope*slope+1); //=-ab/(a^2+1) -> 맵 중앙 원점 기준 수직발의 x좌표
    perp_y = (-slope*(l[0]-MAP_WIDTH/2)+(l[1]-MAP_HEIGHT/2))/(slope*slope+1); //=b/(a^2+1) -> 맵 중앙 원점 기준 수직발의 y좌표
    perp = sqrt(pow(perp_x,2)+pow(perp_y,2)); // -> 맵 중앙 원점 기준 수직발 길이
    slope = atanf(slope);
  }
  length = sqrt(pow(l[3]-l[1],2)+pow(l[2]-l[0],2));

  line_info.push_back(slope);//선분의 경사(rad단위 각도값).
//그러나 범위는 -90도에서 +90도까지만

  line_info.push_back(perp_x);
  line_info.push_back(perp_y);
  line_info.push_back(perp);
  line_info.push_back(length);//선분의 길이
  return line_info;
}









void rectangular_map(vector<Vec4i> lines, float length_threshold, float angle_threshold){

    float slope;
    float length;
    float perp;
    float perp_x;
    float perp_y;
    Vec4i l;

    pos_x=robot_pos.x;
    pos_y=robot_pos.y;
    pos_o=robot_pos.o;

    int oricaseNo, linecaseNo;
    vector<float> oridata, xdata, ydata;



//Debugging: cout<<"input xyo is "<<pos_x<<"/"<<pos_y<<"/"<<pos_o<<endl;

    for(int i = 0; i < lines.size(); i++)
    {
      l = lines[i];
      vector<float> la;
      la = lineAnalysis(l); //검출하여 저장한 lines의 각 선분 l에 대해 분석
      slope =abs(la[0]);
      perp_x = la[1];
      perp_y = la[2];
      perp = abs(la[3]);



      if( 0<pos_o && pos_o<M_PI/2 ){
        oricaseNo=1;
      }else if( M_PI/2<pos_o && pos_o<M_PI ){
        oricaseNo=2;
      }else if(M_PI<pos_o && pos_o<1.5*M_PI  ){
        oricaseNo=3;
      }else if(( 1.5*M_PI<pos_o && pos_o<2*M_PI  )){
        oricaseNo=4;
      }

      if( perp_x>0 && perp_y >0 ){
        linecaseNo=4;
      }else if( perp_x<0 && perp_y >0 ){
        linecaseNo=3;
      }else if( perp_x<0 && perp_y <0 ){
        linecaseNo=2;
      }else if(( perp_x>0 && perp_y <0 )){
        linecaseNo=1;
      }

//Debugging: cout<<"/"<<oricaseNo<<"/"<<linecaseNo<<"/"<<slope<<"/"<<perp<<"/"<<endl;

      switch (oricaseNo){

        case 1:
          switch (linecaseNo){
            case 1:
              if ( (abs(slope-pos_o)<angle_threshold) && (abs(perp-(300-pos_y))<length_threshold) ){
                oridata.push_back(slope);
                ydata.push_back(300-perp);
              }
              break;
            case 2:
              if ( (abs(slope-(M_PI/2-pos_o))<angle_threshold )&&( abs(perp-(pos_x))<length_threshold )){
                oridata.push_back(M_PI/2-slope);
                xdata.push_back(perp);
              }
              break;
            case 3:
              if ( abs(slope-pos_o)<angle_threshold && (abs(perp-(pos_y))<length_threshold)  ){
                oridata.push_back(slope);
                ydata.push_back(perp);
              }
              break;
            case 4:
              if ( abs(slope-(M_PI/2-pos_o))<angle_threshold && (abs(perp-(500-pos_x))<length_threshold) ){
                oridata.push_back(M_PI/2-slope);
                xdata.push_back(500-perp);
              }
          }
          break;

        case 2:
          switch (linecaseNo){
            case 1:
              if ( (abs(slope- (pos_o-M_PI/2))<angle_threshold )&& abs(perp-pos_x)<length_threshold  ){
                oridata.push_back(slope+M_PI/2);
                xdata.push_back(perp);
              }
              break;
            case 2:
              if ( abs(slope-(M_PI-pos_o))<angle_threshold && abs(perp-(pos_y))<length_threshold  ){
                oridata.push_back(M_PI-slope);
                ydata.push_back(perp);
              }
              break;
            case 3:
              if ( abs(slope-(pos_o-M_PI/2))<angle_threshold && abs(perp-(500-pos_x))<length_threshold  ){
                oridata.push_back(slope+M_PI/2);
                xdata.push_back(500-perp);
              }
              break;
            case 4:
              if ( abs(slope-(M_PI-pos_o))<angle_threshold && abs(perp-(300-pos_y))<length_threshold  ){
                oridata.push_back(M_PI-slope);
                ydata.push_back(300-perp);
              }
              break;
            }
              //pos_o가 실제보다 큰 경우
              //아예 큰 오차가 없으면 아무것도 추가가 안되고 -> 이 경우, pos_o를 경곗값 기준 현재상태와 반대방향으로 내보낸다.
              // 맞는 포지션으로 갈 경우 추가가 되며 각도 피드백이 된다.

              //pos_o가 실제보다 작을 경우
              //큰 오차 없으면 아무것도 추가 안되고
              //


          break;

        case 3:
          switch (linecaseNo){
            case 1:
              if ( abs(slope-(pos_o-M_PI))<angle_threshold && abs(pos_y-perp)<length_threshold  ){
                oridata.push_back(slope+M_PI);
                ydata.push_back(perp);
              }
              break;
            case 2:
              if ( abs(slope-(1.5*M_PI-pos_o))<angle_threshold && abs(pos_x-(500-perp))<length_threshold  ){
                oridata.push_back(1.5*M_PI-slope);
                xdata.push_back(500-perp);
              }
              break;
            case 3:
              if ( abs(slope-(pos_o-M_PI))<angle_threshold && abs(pos_y-(300-perp))<length_threshold  ){
                oridata.push_back(slope+M_PI);
                ydata.push_back(300-perp);
              }
              break;
            case 4:
              if ( abs(slope- (1.5*M_PI-pos_o))<angle_threshold && abs(perp-pos_x)<length_threshold  ){
                oridata.push_back(1.5*M_PI-slope);
                xdata.push_back(perp);
              }
              break;
          }
          break;

        case 4:
          switch (linecaseNo){
            case 1:
              if ( abs(slope-(pos_o-1.5*M_PI))<angle_threshold && abs(pos_x-(500-perp) )<length_threshold ){
                oridata.push_back(slope+1.5*M_PI);
                xdata.push_back(500-perp);
              }
              break;
            case 2:
              if ( abs(slope-(2*M_PI-pos_o))<angle_threshold && abs(pos_y-(300-perp))<length_threshold  ){
                oridata.push_back(2*M_PI-slope);
                ydata.push_back(300-perp);
              }
              break;
            case 3:
              if ( abs(slope-(pos_o-1.5*M_PI))<angle_threshold && abs(pos_x-perp)<length_threshold  ){
                oridata.push_back(slope+1.5*M_PI);
                xdata.push_back(perp);
              }
              break;
            case 4:
              if ( abs(slope- (2*M_PI-pos_o))<angle_threshold && abs(perp-pos_y)<length_threshold ){
                oridata.push_back(2*M_PI-slope);
                ydata.push_back(perp);
              }
              break;
          }break;
      }
    }

//Debugging: cout<<"oridata/x/y/line number "<<oridata.size()<<"/"<<xdata.size()<<"/"<<ydata.size()<<"/"<<lines.size()<<endl;

    int button=1;
    if(lines.size()>0){


      if(oridata.size()<lines.size()/5){//pos_o가 잘못된 경우.
      //oridata 사이즈의 기준 크기가 작을수록 밑의 기준은 커야한다. 반대로 기준 크기가 클수록 밑 기준은 작아야한다.
      //+빨리 회전할수록 밑의 기준이 커야 한다
        if(abs(pos_o-M_PI/2)<0.1){//이 크기가 클수록 angle_theshold도 커야 한다.
          pos_o=M_PI/2+(M_PI/2-pos_o); //
          //pos_y=vectorMean(ydata);
        }else if(abs(pos_o-M_PI)<0.1){//0.05가 크면 엄한 녀석을 잡아넣을 수 있다. 그러나 0.05가 작으면 0.05보다 살짝 벗어나는 곳에서 멈추면 잡을수가 없다.
        //angle_threshold의 1/2
          pos_o=M_PI+(M_PI-pos_o);
          //pos_x=vectorMean(xdata);
        } else if(abs(pos_o-1.5*M_PI)<0.1){
          pos_o=1.5*M_PI+(1.5*M_PI-pos_o);
          //pos_y=vectorMean(ydata);
        } else if(abs(pos_o)<0.1){
          pos_o=2*M_PI-pos_o;
          button=0;
        } else if(abs(pos_o-2*M_PI)<0.1&& button){
          pos_o=(2*M_PI-pos_o);
        }



//Debugging: cout<<"Oops"<<endl;

      }else if(xdata.size()<lines.size()/5){
        pos_x=pos_x;
      }else if(ydata.size()<lines.size()/5){
        pos_y=pos_y;
      }
      else{
        pos_o=vectorMean(oridata);
        pos_x=vectorMean(xdata);
        pos_y=vectorMean(ydata);
//Debugging: cout<<"So Good"<<endl;
      }
    }

//Debugging:
    cout<<"output xyo is "<<int(pos_x)<<"/"<<int(pos_y)<<"/"<<int(pos_o)<<endl;

    robot_pos.x=pos_x;
    robot_pos.y=pos_y;
    robot_pos.o=pos_o;

}



void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) //LiDAR scan으로부터 lidar 각도에 따른 거리 배정
{
    map_mutex.lock();
    int count = (scan->angle_max -scan->angle_min)/ scan->angle_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
    }
    map_mutex.unlock();
}

// void line_Callback(const std_msgs::Int8::ConstPtr& entrance) //entrance.data는 1이면 entrance zone이라는 뜻이고, 0이면 harvesting zone이라는 의미.
// // entrance zone에서는 init_odom을 0으로 하고, harvesting zone에 들어선 순간부터 init_odom 을 1로 한다.
// {
//   cout<<"line called back"<<endl;
//   if(entrance != NULL){
//     int ent = entrance -> data;
//     if(ent == 1){
//       init_odom = 0;
//     }
//     else if(init_odom == 0 && ent == 0){
//       init_odom = 1;
//     }
//   }
// }
// init_odom=1;


// void control_Callback(const geometry_msgs::Vector3::ConstPtr& control) //Reflecting control input into the expeted current robot position, instead of the old position
// {
//   robot_pos.x=robot_pos.x+control_x;
//   robot_pos.y=robot_pos.y+control_y;

//   if (robot_pos.o<1 && robot_pos.o+control_o<0){
//     robot_pos.o=2*M_PI+(robot_pos.o+control_o);
//   }else if (robot_pos.o>2*M_PI-1 && robot_pos.o+control_o>2*M_PI){
//     robot_pos.o=robot_pos.o+control_o - 2*M_PI;
//   }else{
//     robot_pos.o=robot_pos.o+control_o;
//   }

// }



void ballPos_Callback(const core_msgs::ball_position::ConstPtr& position)
{
    nBalls = position->size;
    for(int i = 0; i < nBalls; i++)
    {
        ball_dist[i] = position->dist[i];
        ball_angle[i] = position->angle[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		}
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_odometry_node");
    ros::NodeHandle nh; //NodeHandle 클래스의 nh 객체 선언
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback); //LiDAR 데이터 받아오기
    ros::Subscriber subBallPos = nh.subscribe<core_msgs::ball_position>("/position", 1000, ballPos_Callback);
    //ros::Subscriber sub1 = nh.subscribe<std_msgs::Int8>("/entrance", 1, line_Callback); //Entrance zone 들어갔는지 여부 받아오기
    //ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Vector3>("/control", 1000, control_callback); //getting control input

    pub = nh.advertise<geometry_msgs::Vector3>("/odometry", 1); //odometry, 즉 robot의 위치를 Vector3로 발행한다.


    cv::Mat zone = cv::Mat::zeros(400, 600, CV_8UC3);
    line(zone, Point(50, 50), Point(550, 50), Scalar(255,255,255), 1);
    line(zone, Point(50, 50), Point(50, 250), Scalar(255,255,255), 1);
    line(zone, Point(550, 50), Point(550, 350), Scalar(255,255,255), 1);
    line(zone, Point(50, 350), Point(550, 350), Scalar(255,255,255), 1);

    circle(zone, Point(270, 130), 10, cv::Scalar(255,255,255), -1);
    circle(zone, Point(230, 270), 10, cv::Scalar(255,255,255), -1);
    circle(zone, Point(400, 260), 10, cv::Scalar(255,255,255), -1);
    circle(zone, Point(550, 200), 10, cv::Scalar(0,255,0), -1);

    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);

        //CV_8UC3는 RGB 3채널 컬러 이미지를 위한 데이터 방식이다. 일단 zero로 initialize 하여 map 객체 만듦



        //Substracting the obstacle data from laser scan, and Drawing remaining wall data on the virtual map
        vector<float> wall_distance, wall_degree;

        wall_distance.push_back(lidar_distance[0]);
        wall_degree.push_back(lidar_degree[0]);
        int is_difference_small=1;

        for(int i = 1; i < lidar_size; i++)
        {

          if(lidar_distance[i] <50){ //smaller than the maximum liDAR range(distance)
              wall_distance.push_back(lidar_distance[i]);
              wall_degree.push_back(lidar_degree[i]);
              is_difference_small++;
          }

          if(abs(lidar_distance[i] - lidar_distance[i-1]) > 2){
          //distance changed abruptly -> outermost wall or an another distant obstacle is detected.

            if(is_difference_small < 20){//threshold should be larger than the maximum view angle of the obstacle
              //this means that formerly detected object has small view angle, which means it is likely to be an obstacle
              for(int j=0; j<is_difference_small; j++){
                wall_distance.pop_back();
                wall_degree.pop_back();
              } //remove the added lidar data as much as the view angle of the obstacle
            }
            //If not, the former object was not obstacle, but wall.

            is_difference_small=1; //Initiallization
          }
        }

        //Drawing
        for(int i = 1; i<wall_distance.size(); i++){
          int cxi = MAP_WIDTH/2 + (int)(wall_distance[i]*sin(wall_degree[i])/MAP_RESOL);
          int cyi = MAP_HEIGHT/2 + (int)(wall_distance[i]*cos(wall_degree[i])/MAP_RESOL);
          int cxi0 = MAP_WIDTH/2 + (int)(wall_distance[i-1]*sin(wall_degree[i-1])/MAP_RESOL);
          int cyi0 = MAP_HEIGHT/2 + (int)(wall_distance[i-1]*cos(wall_degree[i-1])/MAP_RESOL);

          if(check_point_range(cxi,cyi) && check_point_range(cxi0,cyi0) && sqrt(pow(cxi-cxi0,2)+pow(cyi-cyi0,2)) < 0.6/MAP_RESOL)
          {
            line(map, Point(cxi, cyi), Point(cxi0, cyi0), Scalar(255,255,255), 1); //line 함수는 (그릴 대상, 양 끝점1,양끝점2, 선 색깔, 선 타입)으로 입력하여 선을 그려줌
          }
        }

        cv::Mat gray; //행렬 클래스. 영상/이미지 저장에도 쓰임.
        cvtColor(map,gray,COLOR_BGR2GRAY); //map을 회색으로 칠해(?) gray라는 MAT 클래스로 변환

        cv::Mat edges;
        Canny(gray,edges,20,200);
        //gray로부터 경곗값 50, 150을 기준으로 이미지의 경계선만을 검출하여 edge라는 행렬로 출력.
        vector<Vec4i> lines; //lines는 밑의 Hough 변환 결과를 받아올 array. 선분의 시작점 좌표 x,y와 끝점좌표 x,y를 받아옴.

        HoughLinesP(edges, lines, 0.5, CV_PI/180, 30, 20, 50);
        //주어진 이미지 gray로부터 직선 검출. 1 과 CV_PI/180는 모델링할 직선 방정식 r=xcos(th)+ysin(th)의 파라미터 r과 th의 해상도 개념.
        //30은 선으로 치려면 최소 몇 개 데이터 이상이어야 하는지, 15는 선으로 검출하기 위한 최소 길이, 5는 다른 선으로 간주되지 않기 위한 점 데이터 사이 최대 허용 길이이다.


        rectangular_map(lines, 20, 0.2); //angle_threshold는 최대 0.7보다는 작아야 한다.



//Debugging: circle(map,Point(MAP_WIDTH/2,MAP_HEIGHT/2),10, cv::Scalar(0,0,255), -1);


        circle(zone, Point(50+int(robot_pos.x), 350-int(robot_pos.y)), 3, cv::Scalar(255,0,0), -1);
        for (int i=0; i<nBalls; i++){ //ball_dist[i], ball_angle[i]
          int ball_x = 50 + (int)(robot_pos.x + ball_dist[i]*cos(ball_angle[i]+robot_pos.o)*100);
          int ball_y = 350 - (int)(robot_pos.y + ball_dist[i]*sin(ball_angle[i]+robot_pos.o)*100);
          if (ball_x>50 && ball_x<550 && ball_y>50 && ball_y<350){
            circle(zone, Point(ball_x, ball_y),2,cv::Scalar(0,0,255), -1);
          }
        }
        cv::imshow("Harvesting zone map",zone);
        cv::waitKey(50);


        // if(init_odom == 0){
        //   pos_x = 0.0;
        //   pos_y = 0.0;
        //   pos_o = 0.0;
        // } 각도가 제일 문제. 1. 허용오차 작게 하고 예상포인트 세트를 다 돌려봐서 그 중 맞는걸로, 2. 처음 시행시에는 허용오차 크게, 3. 들어오기 꽤 전부터 먼저 돌리면서 허용오차는 빡세게 하면서 기다리기
        //이 노드는 아무리 빨라도 |x좌표|+초기 허용오차 <50일 때만 쓸수있다.

      	odometry.x=robot_pos.x;
      	odometry.y=robot_pos.y;
      	odometry.z=robot_pos.o;
      	pub.publish(odometry);

        ros::spinOnce();
    }

    return 0;
}
