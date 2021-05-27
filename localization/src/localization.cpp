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
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int initialization=1;

float MAP_CX = 500; 
float MAP_CY = 500;
float MAP_RESOL = 0.01; 
int MAP_WIDTH = 1000; 
int MAP_HEIGHT = 1000; 
int MAP_CENTER = 50;


geometry_msgs::Vector3 robot_pos;
std_msgs::Float32MultiArray obs_pos;

float pos_x;
float pos_y;
float pos_o;

float control_x;
float control_y;
float control_o;





int lidar_size;//lidar callback에서 사용되는 lidar point 개수
float lidar_degree[400]; 
float lidar_distance[400];



boost::mutex map_mutex;

float pi=2*atanf(1);
#define RAD2DEG(x) ((x)*180./M_PI)




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
    pos_o=robot_pos.z;

    int oricaseNo, linecaseNo;
    vector<float> oridata, xdata, ydata;
    


//Debugging: cout<<endl<<"input xyo is "<<pos_x<<"/"<<pos_y<<"/"<<pos_o<<endl;

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

      if( perp_x>=0 && perp_y >=0 ){
        linecaseNo=4;
      }else if( perp_x<=0 && perp_y >=0 ){
        linecaseNo=3;
      }else if( perp_x<=0 && perp_y <=0 ){
        linecaseNo=2;
      }else if(( perp_x>=0 && perp_y <=0 )){
        linecaseNo=1;
      }



//Debugging: cout<<"CASE "<<oricaseNo<<linecaseNo<<endl;

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
              if( pos_x<50){
                if ( (abs(slope-(M_PI/2-pos_o))<angle_threshold )&&( abs(perp-(100+pos_x))<length_threshold )){
                  oridata.push_back(M_PI/2-slope);
                  xdata.push_back(-100+perp);
              }
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

              if( pos_x<50){
                if ( (abs(slope- (pos_o-M_PI/2))<angle_threshold )&&( abs(perp-(100+pos_x))<length_threshold )){
                  oridata.push_back(slope+M_PI/2);
                  xdata.push_back(-100+perp);
              }
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

              if( pos_x<50){
                if ( (abs(slope- (1.5*M_PI-pos_o))<angle_threshold )&&( abs(perp-(100+pos_x))<length_threshold )){
                  oridata.push_back(1.5*M_PI-slope);
                  xdata.push_back(-100+perp);
              }
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

              if( pos_x<50){  
                if ( (abs(slope-(pos_o-1.5*M_PI))<angle_threshold )&&( abs(perp-(100+pos_x))<length_threshold )){
                  oridata.push_back(slope+1.5*M_PI);
                  xdata.push_back(-100+perp);
              }
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
        }else if(abs(pos_o-M_PI)<0.1){//0.05가 크면 엄한 녀석을 잡아넣을 수 있다. 그러나 0.05가 작으면 0.05보다 살짝 벗어나는 곳에서 멈추면 잡을수가 없다.
        //angle_threshold의 1/2
          pos_o=M_PI+(M_PI-pos_o);
        } else if(abs(pos_o-1.5*M_PI)<0.1){
          pos_o=1.5*M_PI+(1.5*M_PI-pos_o);
        } else if(0<pos_o && pos_o<0.1){
          pos_o=2*M_PI-pos_o;
          button=0;
        }else if(0>pos_o){
          pos_o=2*M_PI+pos_o;
          button=0;
        } else if( 0<2*M_PI-pos_o && 2*M_PI-pos_o<0.1 && button){
          pos_o=(2*M_PI-pos_o);
        } else if(2*M_PI-pos_o<0  && button){
          pos_o=pos_o-2*M_PI;
        }

        if(xdata.size()<2 && ydata.size()>2){
          pos_x=pos_x;
          pos_y=vectorMean(ydata);
        }else if(ydata.size()<3 && xdata.size()>1 ){
          pos_y=pos_y;
          pos_x=vectorMean(xdata);
        }else if(ydata.size()<3 && xdata.size()<2){
          pos_x=pos_x;
          pos_y=pos_y;
        }


      }else if(xdata.size()<2){
          pos_x=pos_x;
          pos_y=vectorMean(ydata);
          pos_o=vectorMean(oridata);
      }else if(ydata.size()<3){
          pos_y=pos_y;
          pos_x=vectorMean(xdata);
          pos_o=vectorMean(oridata);
      }else{
        pos_o=vectorMean(oridata);
        pos_x=vectorMean(xdata);
        pos_y=vectorMean(ydata);
      }
    }





//Debugging:
cout<<"output xyo is "<<pos_x<<"/"<<pos_y<<"/"<<pos_o<<endl;

    robot_pos.x=pos_x;
    robot_pos.y=pos_y;
    robot_pos.z=pos_o;
    
}



void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) //LiDAR scan으로부터 lidar 각도에 따른 거리 배정
{
    map_mutex.lock();
    int count = (scan->angle_max -scan->angle_min)/ scan->angle_increment+1;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
    }
    map_mutex.unlock();
}



int zone_info;
void entrance_Callback(const std_msgs::Int8::ConstPtr& zone)
{
  zone_info=zone->data;

}

float linear_vel;
float angular_vel;
float Ts;

void control_input_Callback(const geometry_msgs::Twist::ConstPtr& targetVel){
  linear_vel = targetVel->linear.x;
  angular_vel = targetVel->angular.z;
  
  Ts=0.001;
  if (zone_info==2 && initialization>10){
  robot_pos.x=robot_pos.x+linear_vel*cos(robot_pos.z)*Ts;
  robot_pos.y=robot_pos.y+linear_vel*sin(robot_pos.z)*Ts;
  robot_pos.z=robot_pos.z+angular_vel*Ts;
  }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh; //NodeHandle 클래스의 nh 객체 선언
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback); //LiDAR 데이터 받아오기
    ros::Subscriber sub1 = nh.subscribe<std_msgs::Int8>("/zone", 1, entrance_Callback); //Entrance zone 들어갔는지 여부 받아오기
    ros::Subscriber commandVel = nh.subscribe<geometry_msgs::Twist>("/command_vel", 10, control_input_Callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/robot_pos", 1); //odometry, 즉 robot의 위치를 Vector3로 발행한다.
    ros::Publisher pub1 = nh.advertise<std_msgs::Float32MultiArray>("/obs_pos", 1);


// Debugging:
// cv::Mat zone = cv::Mat::zeros(600, 600, CV_8UC3);
// line(zone, Point(50, 50), Point(550, 50), Scalar(255,255,255), 1);
// line(zone, Point(50, 50), Point(50, 250), Scalar(255,255,255), 1);
// line(zone, Point(550, 50), Point(550, 350), Scalar(255,255,255), 1);
// line(zone, Point(50, 350), Point(550, 350), Scalar(255,255,255), 1);

// circle(zone, Point(270, 130), 10, cv::Scalar(255,255,255), -1);
// circle(zone, Point(230, 270), 10, cv::Scalar(255,255,255), -1);
// circle(zone, Point(400, 260), 10, cv::Scalar(255,255,255), -1);

// circle(zone, Point(550, 200), 10, cv::Scalar(0,255,0), -1);


    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);

        //Substracting the obstacle data from laser scan, and Drawing remaining wall data on the virtual map
        vector<float> wall_distance, wall_degree;
        vector<float> obs_distance, obs_degree;

        wall_distance.push_back(lidar_distance[0]);
        wall_degree.push_back(lidar_degree[0]);

        int view_angle=0;
        float sum=lidar_distance[0];
        float avg_distance;
        float obstacle_width;


        for(int i = 1; i < lidar_size; i++)
        {

          if(lidar_distance[i] >0){ //smaller than the maximum liDAR range(distance)
              wall_distance.push_back(lidar_distance[i]);
              wall_degree.push_back(lidar_degree[i]);
              view_angle++;
              sum=sum+lidar_distance[i];
          }

          if(abs(lidar_distance[i] - lidar_distance[i-1]) > 0.3){
          //distance changed abruptly -> outermost wall or an another distant obstacle is detected.
            view_angle=view_angle-1;
            sum=sum-lidar_distance[i];
            avg_distance=sum/(view_angle+1);
            obstacle_width=avg_distance*(M_PI/180)*view_angle;
//Debugging:cout<<obstacle_width<<"/"<<view_angle<<endl;

            if(0.07<obstacle_width && obstacle_width < 0.15 && view_angle>0 && abs(lidar_distance[i-1]-avg_distance)<0.2 ){//threshold should be larger than the maximum width of the obstacle
              //this means that formerly detected object has small width, which means it is likely to be an obstacle

              obs_distance.push_back(lidar_distance[i-1-view_angle/2]+0.15/2);
              if (lidar_degree[i-1-view_angle/2]>=M_PI/2 && lidar_degree[i-1-view_angle/2]<=1.5*M_PI){
                obs_degree.push_back(lidar_degree[i-1-view_angle/2]-M_PI/2);
              } else if(lidar_degree[i-1-view_angle/2]<=M_PI/2){
                obs_degree.push_back(lidar_degree[i-1-view_angle/2]-M_PI/2);
              } else if(lidar_degree[i-1-view_angle/2]>=1.5*M_PI && lidar_degree[i-1-view_angle/2]<=2*M_PI){
                obs_degree.push_back(lidar_degree[i-1-view_angle/2]-2.5*M_PI);
              }


              for(int j=0; j<view_angle+2; j++){
                wall_distance.pop_back();
                wall_degree.pop_back();
              } //remove the added lidar data as much as the view angle of the obstacle
              wall_distance.push_back(lidar_distance[i]);
              wall_degree.push_back(lidar_degree[i]);


            }
              view_angle=0; //Initiallization
              sum=lidar_distance[i];

          }

        }
        
        //Drawing

// Debugging:
// int cxi0 = MAP_WIDTH/2 + (int)(wall_distance[0]*sin(wall_degree[0])/MAP_RESOL);
// int cyi0 = MAP_HEIGHT/2 + (int)(wall_distance[0]*cos(wall_degree[0])/MAP_RESOL);
// circle(map, Point(cxi0, cyi0), 3, cv::Scalar(255,0,0), -1);


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
        Canny(gray,edges,20,200);//gray로부터 경곗값 50, 150을 기준으로 이미지의 경계선만을 검출하여 edge라는 행렬로 출력.
        vector<Vec4i> lines; //lines는 밑의 Hough 변환 결과를 받아올 array. 선분의 시작점 좌표 x,y와 끝점좌표 x,y를 받아옴.
        HoughLinesP(edges, lines, 0.5, CV_PI/180, 20, 20, 90);
        //주어진 이미지 gray로부터 직선 검출. 1 과 CV_PI/180는 모델링할 직선 방정식 r=xcos(th)+ysin(th)의 파라미터 r과 th의 해상도 개념.
        //30은 선으로 치려면 최소 몇 개 데이터 이상이어야 하는지, 15는 선으로 검출하기 위한 최소 길이, 5는 다른 선으로 간주되지 않기 위한 점 데이터 사이 최대 허용 길이이다.



    Vec4i l;
         for(int i = 0; i < lines.size(); i++)
    {
      l = lines[i];
      line(map, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1);
    }




//테스트 용으로 사용시 아래 단락을 주석처리하고 robot_pos의 초기값을 main함수 밖에서 설정
        if (zone_info==1){
          robot_pos.x=-30;
          robot_pos.y=50;
          robot_pos.z=2*M_PI-0.4;
          cout<<"localization not yet"<<endl;
        }else if (zone_info==2 && initial_step<10){
          rectangular_map(lines, 25, 0.3);
          initial_step++;
          if( robot_pos.x==-30 && robot_pos.y==50 && robot_pos.z==2*M_PI-0.4){
            robot_pos.x=-30;
            robot_pos.y=50;
            robot_pos.z=2*M_PI-0.09;
            rectangular_map(lines, 25, 0.3);
          }

        }else{
          rectangular_map(lines, 20, 0.2); //angle_threshold는 최대 0.7보다는 작아야 한다.
        }
        
        

        //Getting obstacle location 
        obs_pos.data.clear();
        
        if(zone_info==2){

            for (int i=0; i<obs_distance.size(); i++){
              obs_pos.data.push_back(obs_distance[i]);
              obs_pos.data.push_back(obs_degree[i]);
            }
        }

//Debugging:
// for(int i=0; i<obs_pos.data.size()/2; i++ ){
//   //cout<<obs_pos.data[2*i]<<"/"<<obs_pos.data[2*i+1]<<endl;
// }

// Debugging: circle(map,Point(MAP_WIDTH/2,MAP_HEIGHT/2),10, cv::Scalar(0,0,255), -1);   
// circle(zone, Point(50+int(robot_pos.x), 350-int(robot_pos.y)), 3, cv::Scalar(255,0,0), -1);
// cv::imshow("Harvesting zone map",zone);
// cv::imshow("Harvesting zone map",map);
// cv::waitKey(50);

      	pub.publish(robot_pos);
        pub1.publish(obs_pos);

        ros::spinOnce();
    }

    return 0;
}
