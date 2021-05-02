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

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//Map의 역할은??

float MAP_CX = 350.5; //MAP의 최대 X좌표 크기->MAP의 정중앙을 원점으로 치는듯?
float MAP_CY = 350.5;
float MAP_RESOL = 0.01; //MAP의 최소 격자 크기
int MAP_WIDTH = 700; //MAP의 가로방향 전체 크기
int MAP_HEIGHT = 700; //MAP의 세로방향 전체 크기
int MAP_CENTER = 50;
//???
int OBSTACLE_PADDING = 2;
//장애물 크기->왜 2?
int OBSTACLE_CONNECT_MAX = 15;
//???

geometry_msgs::Vector3 odometry;
ros::Publisher pub;

int init_odom = 1; // Entrance node도 켰으면, 0으로 초기화!!

int lidar_size;//lidar callback에서 사용되는 lidar point 개수
float lidar_degree[400]; //넉넉하게 400ㅇ로 한듯?
float lidar_distance[400];

float cyl_x[2] = {4.0,5.5};// cyl은 기둥 의미
float cyl_y[2] = {1.5,0.7};

float pos_x = 0.0;
float pos_y = 0.0;
float pos_o = 0.0;

float ref_x = 0.0;
float ref_y = 0.0;
float ref_o = 0.0;
//??

boost::mutex map_mutex;

#define RAD2DEG(x) ((x)*180./M_PI)

bool check_point_range(int cx, int cy) //어떤 input 위치가 MAP 안에 있는지 검사
{
    return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}

float vectorMean(std::vector<float> numbers){
    if(numbers.empty()) return 0;
    float total = 0;
    for (int number : numbers){
        total += number;
    }
    return (total /(float)numbers.size());
}

float vectorVariance(float mean, std::vector<float> numbers)
{
    float result = 0;
    for(int number : numbers){
        result += (number-mean)*(number - mean);
    }
    return sqrt(result/(float)numbers.size());
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















vector<float> cylinderOdometry(vector<float> cyl_rel_x_gAP, vector<float> cyl_rel_y_gAP){

//???

  vector<float> abs;
  if(cyl_rel_x_gAP.size() <= 1 || cyl_rel_x_gAP.size() > 4){
    abs.push_back(pos_x);
    abs.push_back(pos_y);
    abs.push_back(pos_o);
    return abs;
  }

  float closest[2] = {10,10};
  float second_closest[2] = {10,10};

  for(int i = 0; i<cyl_rel_x_gAP.size(); i++){
    if(pow(cyl_rel_x_gAP[i],2) + pow(cyl_rel_y_gAP[i],2) < pow(second_closest[0],2) + pow(second_closest[1],2)){
      second_closest[0] = cyl_rel_x_gAP[i];
      second_closest[1] = cyl_rel_y_gAP[i];
    }

    if(pow(second_closest[0],2) + pow(second_closest[1],2)<pow(closest[0],2) + pow(closest[1],2)){
      float to_second_x = closest[0];
      float to_second_y = closest[1];
      closest[0] = second_closest[0];
      closest[1] = second_closest[1];
      second_closest[0] = to_second_x;
      second_closest[1] = to_second_y;
    }
  }

  float closest_rad[2];
  float second_closest_rad[2];
  closest_rad[0] = sqrtf(pow(closest[0],2)+pow(closest[1],2));
  closest_rad[1] = atan2f(closest[1],closest[0]);
  second_closest_rad[0] = sqrtf(pow(second_closest[0],2)+pow(second_closest[1],2));
  second_closest_rad[1] = atan2f(second_closest[1],second_closest[0]);

  // trigonometry
  float opposite_side, opposite_side_angle;
  opposite_side = sqrtf(pow(cyl_x[1]-cyl_x[0],2)+pow(cyl_y[1]-cyl_y[0],2));
  opposite_side_angle = atan2f((cyl_y[1]-cyl_y[0]),(cyl_x[1]-cyl_x[0]));

  float between_angle;
  between_angle = fabs(second_closest_rad[1]-closest_rad[1]);

  float closest_angle;
  closest_angle = acos((pow(closest_rad[0],2)+pow(opposite_side,2)-pow(second_closest_rad[0],2))/(2*closest_rad[0]*opposite_side));

  float x_abs = cyl_x[0] + closest_rad[0]*cos(opposite_side_angle-closest_angle);
  float y_abs = cyl_y[0] + closest_rad[0]*sin(opposite_side_angle-closest_angle);
  float ori_abs = atan2(cyl_y[0]-y_abs,cyl_x[0]-x_abs)-closest_rad[1];
  abs.push_back(x_abs);
  abs.push_back(y_abs);
  if(pos_o == 0){
    abs.push_back(ori_abs);
  }
  else{
    abs.push_back(pos_o);
  }
  cout<<"cylOdom : "<<abs[0]<<" , "<<abs[1]<<" , "<<abs[2]<<endl;
  return abs;
  }











void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) //LiDAR scan으로부터 lidar 각도에 따른 거리 배정
{
    map_mutex.lock();
    int count = scan->angle_max / scan->angle_increment;
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











int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_odometry_node");
    ros::NodeHandle nh; //NodeHandle 클래스의 nh 객체 선언
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback); //LiDAR 데이터 받아오기

    //ros::Subscriber sub1 = nh.subscribe<std_msgs::Int8>("/entrance", 1, line_Callback); //Entrance zone 들어갔는지 여부 받아오기

    pub = nh.advertise<geometry_msgs::Vector3>("/odometry", 1); //odometry, 즉 robot의 위치를 Vector3로 발행한다.

    while(ros::ok){
        cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);

        //CV_8UC3는 RGB 3채널 컬러 이미지를 위한 데이터 방식이다. 일단 zero로 initialize 하여 map 객체 만듦

        float obstacle_x, obstacle_y;
        int cx, cy;
        int cx1, cx2, cy1, cy2;

        float right_end_distance = 0;
        float right_end_degree = 0;
        float left_end_distance = 0;
        float left_end_degree = 0; //기둥의 왼쪽/오른쪽 envelope
        int cyl_pts = 0; //기둥에 해당하는 라이다의 포인트 개수
        int cyl_number = 0; //기둥 개수

        int is_cyl = 0;//기둥 감지 여부 판별
        vector<float> cyl_rel_x, cyl_rel_y; //cyl_rel_x와 y는 기둥의 로봇에 대한 상대적 위치를 말함




        vector<float> wall_distance, wall_degree; //외벽 거리 및 위치 저장

        for(int i = 1; i < lidar_size-1; i++)
        {
          if(lidar_distance[i] < 8){ //8 이하면 wall에 일단 추가
          //-> 8이상이면 어떡함?
            wall_distance.push_back(lidar_distance[i]);
            wall_degree.push_back(lidar_degree[i]);
          }
            //Column Detected
            if(lidar_distance[i-1] - lidar_distance[i] > 0.6){ //Threshold 값 왜 0.6? -> 벽에서 갑자기 기둥으로 바뀌면 값차이 클 것. lidar는 ccw 방향
              right_end_distance = lidar_distance[i];//
              right_end_degree = lidar_degree[i];
              cyl_pts = 0;
              is_cyl = 1;
            }

            if(is_cyl == 1){
              cyl_pts++;
            }

            if(lidar_distance[i+1] - lidar_distance[i] > 0.6){
              if(is_cyl == 1 && cyl_pts<15 && cyl_pts>1){ //기둥이 감지됐고, 기둥에 해당하는 포인트 개수가 2~14 사이였다면 이제 기둥을 벗어난 것으로 친다.
                left_end_distance = lidar_distance[i];
                left_end_degree = lidar_degree[i];

                float bow, bow_angle;
                bow = sqrtf(pow(right_end_distance,2)+pow(left_end_distance,2)-2*right_end_distance*left_end_distance*cos(left_end_degree-right_end_degree));
                bow_angle = cos(bow/(0.14));
          //bow는 기둥의 너비, bow_angle은 ??

                float angle1, angle2;
                angle1 = acos((pow(right_end_distance,2)+pow(bow,2)-pow(left_end_distance,2))/(2*right_end_distance*bow));
                angle2 = angle1 + bow_angle;
          //angle1은 ??

                float cyl_distance, cyl_degree;
                cyl_distance = sqrtf(pow(right_end_distance,2)+0.0049-0.14*right_end_distance*cos(angle2));
                cyl_degree = acos((pow(right_end_distance,2)+pow(cyl_distance,2)-0.0049)/(2*right_end_distance*cyl_distance)) + right_end_degree;

          //cyl은 기둥을 말함. cyl_dist는
                obstacle_x = cyl_distance*cos(cyl_degree);
                obstacle_y = cyl_distance*sin(cyl_degree);

                // obstacle도 기둥을 말하는데, obstacle_x, y는 로봇기준 기둥의 x,y좌표임.
                cyl_rel_x.push_back(obstacle_x);
                cyl_rel_y.push_back(obstacle_y);
                cyl_number++;
                //cyl은 기둥이고, cyl_rel_x와 y는 기둥의 로봇에 대한 상대적 위치를 말함. 기둥 개수 추가

                //cout<<"COL "<<cyl_number<<" Angle = "<<cyl_degree*180/3.14159265359<<" Distance = "<<cyl_distance<<" Points = "<<cyl_pts<<endl;
                cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
                cy = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
                //cv::circle(map,cv::Point(cx, cy),(int)(0.07/MAP_RESOL), cv::Scalar(0,0,255), 1);

                is_cyl = 0;
                right_end_distance = 0;
                right_end_degree = 0;
                left_end_distance = 0;
                left_end_degree = 0; //다시 초기화

                for(int j=0; j<cyl_pts; j++){
                  wall_distance.pop_back();
                  wall_degree.pop_back();
                } //기둥에 해당하는 포인트 개수만큼 wall 정보에서 제거
              }
              else //기둥이 감지되지 않았거나, 기둥에 해당하는 포인트 개수가 2~14 사이가 아니라면 거리차이가 0.6보다 커도 기둥이 아닌 것으로 친다.
              {
                is_cyl = 0;
                right_end_distance = 0;
                right_end_degree = 0;

              }
            }
        }

        for(int i = 1; i<wall_distance.size(); i++){ //wall 정보로 추가된 포인트 개수만큼 실행
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
        Canny(gray,edges,50,150);
        //gray로부터 경곗값 50, 150을 기준으로 이미지의 경계선만을 검출하여 edge라는 행렬로 출력.
        vector<Vec4i> lines; //lines는 밑의 Hough 변환 결과를 받아올 array. 선분의 시작점 좌표 x,y와 끝점좌표 x,y를 받아옴.

        HoughLinesP(edges, lines, 1, CV_PI/180, 30, 15, 5 );
        //주어진 이미지 gray로부터 직선 검출. 1 과 CV_PI/180는 모델링할 직선 방정식 r=xcos(th)+ysin(th)의 파라미터 r과 th의 해상도 개념.
        //30은 선으로 치려면 최소 몇 개 데이터 이상이어야 하는지, 15는 선으로 검출하기 위한 최소 길이, 5는 다른 선으로 간주되지 않기 위한 점 데이터 사이 최대 허용 길이이다.

        float major_slope;
        float minor_slope;
        int num_a = 0;
        float cos_a = 0;
        float sin_a = 0;
        float slope_a = 0;
        float perp_a;
        int num_b = 0;
        float cos_b = 0;
        float sin_b = 0;
        float slope_b = 0;
        float perp_b;
        int major = 1;

        int slope_number = 0;
        float slope;
        float length;
        float perp;
        float perp_x;
        float perp_y;

        Vec4i l;

        for(int i = 0; i < lines.size(); i++)
        {
          l = lines[i];
          vector<float> lA;
          lA = lineAnalysis(l); //검출하여 저장한 lines의 각 선분 l에 대해 분석
          slope = lA[0];
          perp_x = lA[1];
          perp_y = lA[2];
          perp = lA[3];


          if(num_a == 0){
            slope_a = slope;
            cos_a = cosf(slope);
            sin_a = sinf(slope);
            perp_a = perp;
            line(map, Point(l[0], l[1]), Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y), Scalar(0,255,0), 1);


            cv::circle(map,Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y),2, cv::Scalar(0,255,0), -1); //수직발을 중심으로 반지름 2 원
//왜 그리지?
            num_a++;
          }
          else{
            if(abs(slope-slope_a) < atan(0.05)){ //새 선분의 slope가 이전 slope와 차이가 거의 없으면
              slope_a = (slope_a*num_a + slope)/(float)(num_a+1); //이전 slope와 num_a:1로 내분점을 새 slope로 한다.
//왜?? 이 파트는 왜 있는 걸까

              line(map, Point(l[0], l[1]), Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y), Scalar(0,255,0), 1);
              cv::circle(map,Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y),2, cv::Scalar(0,255,0), -1);

              if(abs(abs(perp-perp_a)*MAP_RESOL - 3) < 0.2){
  //새 perp와 이전 perp의 차이가 300 정도이면
                major = 1;
              }
              else if(abs(abs(perp-perp_a)*MAP_RESOL - 5) < 0.2 || abs(abs(perp-perp_a)*MAP_RESOL - 1.0) < 0.2){
  //새 perp와 전 perp 차이가 500 정도거나 100 정도면
                major = 2;
              }
              num_a++;
            }
            else if(abs(abs(slope-slope_a)-2*atan(1))<atan(0.05)){
              //새 선분의 slope와 이전 slope 차이가 대충 90도면
              //밑에는 비슷함
              if(num_b == 0){
                slope_b = slope;
                cos_b = cosf(slope);
                sin_b = sinf(slope);
                perp_b = perp;
                line(map, Point(l[0], l[1]), Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y), Scalar(0,0,255), 1);
                cv::circle(map,Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y),2, cv::Scalar(0,0,255), -1);
//여긴 num_b가 없음. 왜??
              }
              else{
                slope_b = (slope_b*num_b + slope)/(float)(num_b+1);
                line(map, Point(l[0], l[1]), Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y), Scalar(0,0,255), 1);
                cv::circle(map,Point(MAP_WIDTH/2+perp_x, MAP_HEIGHT/2+perp_y),2, cv::Scalar(0,0,255), -1);
                if(abs(abs(perp-perp_b)*MAP_RESOL - 3) < 0.2){
                  major = 2;
                }
                else if(abs(abs(perp-perp_a)*MAP_RESOL - 5) < 0.2 || abs(abs(perp-perp_a)*MAP_RESOL - 1.0)< 0.2){
                  major = 1;
                }
              }
              num_b++;
            }
          }
        }


        //여기부터 orientation 잡기
        float cand_o1;
        float cand_o2;
        float cand_o3;
        float cand_o4;
        if(num_a <= 1){
//위에서 맨 처음 부분만 실행된 경우?
//major의 의미는?
          major = 0;
        }
        else if(major == 1){ //
          major_slope = slope_a; //네 벽면 중 한 벽면
          minor_slope = slope_b; // 위의 벽면과 수직한 면
          cand_o1 = major_slope + 2*atan(1);
//왜 major slope?
          cand_o2 = major_slope - 2*atan(1); //두 가지 가능성(+180도 -180도)

          if(cosf(cand_o1-pos_o)>0.5){ //여기서 일단 pos_o는 0인듯?
            pos_o = cand_o1;
          }
//??
          else if(cosf(cand_o2-pos_o)>0.5){
            pos_o = cand_o2;
          }
        }
        else if(major == 2){
          major_slope = slope_b;
          minor_slope = slope_a;
          cand_o1 = major_slope + 2*atan(1);
          cand_o2 = major_slope - 2*atan(1);

          if(cosf(cand_o1-pos_o)>0.5){
            pos_o = cand_o1;
          }
          else if(cosf(cand_o2-pos_o)>0.5){
            pos_o = cand_o2;
          }
        }
        else{
          cand_o1 = slope_a + 2*atan(1);
          cand_o2 = slope_a - 2*atan(1);
          cand_o3 = slope_b + 2*atan(1);
          cand_o4 = slope_b - 2*atan(1);

          if(cosf(cand_o1-pos_o)>0.75){
            pos_o = cand_o1;
          }
          else if(cosf(cand_o2-pos_o)>0.75){
            pos_o = cand_o2;
          }
          else if(cosf(cand_o3-pos_o)>0.75){
            pos_o = cand_o3;
          }
          else if(cosf(cand_o4-pos_o)>0.75){
            pos_o = cand_o4;
          }
        }



      //여기부터 robot x,y 좌표 잡기
        vector<float> lx_perp, lxo_perp, sx_perp, sxo_perp;
        vector<float> lx_x, lxo_x, sx_x, sxo_x;
        vector<float> lx_y, lxo_y, sx_y, sxo_y;

        for(int i = 0; i < lines.size(); i++)
        {
          l = lines[i];
          vector<float> lA = lineAnalysis(l);
          slope = lA[0];
          perp_x = lA[1];
          perp_y = lA[2];
          perp = lA[3];

          if(cosf(atan2f(perp_x, perp_y)+pos_o)>0.96){//
            lx_perp.push_back(perp);
            lx_x.push_back(l[0]);
            lx_x.push_back(l[2]);
            lx_y.push_back(l[1]);
            lx_y.push_back(l[3]);
          }
          else if(sinf(atan2f(perp_x, perp_y)+pos_o)>0.96){
            sx_perp.push_back(perp);
            sx_x.push_back(l[0]);
            sx_x.push_back(l[2]);
            sx_y.push_back(l[1]);
            sx_y.push_back(l[3]);
          }
          else if(cosf(atan2f(perp_x, perp_y)+pos_o)<-0.96){
            lxo_perp.push_back(perp);
            lxo_x.push_back(l[0]);
            lxo_x.push_back(l[2]);
            lxo_y.push_back(l[1]);
            lxo_y.push_back(l[3]);
          }
          else if(sin(atan2f(perp_x, perp_y)+pos_o)<-0.96){
            sxo_perp.push_back(perp);
            sxo_x.push_back(l[0]);
            sxo_x.push_back(l[2]);
            sxo_y.push_back(l[1]);
            sxo_y.push_back(l[3]);
          }
        }

        float sx = 0;
        float sx_o = 0;
        float sx_cos = 0;
        float sx_sin = 0;
        float sx_len = 0;
        if(sx_x.size() > 1){ //sx 케이스에 해당하면
          int x1 = *min_element(sx_x.begin(), sx_x.end()); // x1은 벽 선분 중 x좌표 작은거
          //begin(), end() 함수는 iterator(포인터와 비슷)을 반환한다
          //min_element와 max_element도 iterator(포인터와 비슷)을 반환하므로 *으로 값을 내줘야 한다



          int x2 = *max_element(sx_x.begin(), sx_x.end()); // x2는 벽면선분 양끝점 중 x좌표 큰거

          int y1, y2;

          if(abs(x2-x1)<10){ // x좌표 별 차이 안나면 : 선분이 수직에 가까우면
            y1 = *min_element(sx_y.begin(), sx_y.end());
            y2 = *max_element(sx_y.begin(), sx_y.end());
            //y1은 작은거, y2는 큰거

            x1 = sx_x[min_element(sx_y.begin(), sx_y.end())-sx_y.begin()];
                        //y1(작은거)이 원래 시작점 좌표였으면 x1도 시작점의 좌표로 한다. y1 작은게 끝점 좌표였으면 x1은 끝점 좌표로 한다
            x2 = sx_x[max_element(sx_y.begin(), sx_y.end())-sx_y.begin()];
                        //y2(큰거)가 벽면선분의 시작점 좌표였으면 x2를 시작점의 좌표로
          }
          else{
            y1 = sx_y[min_element(sx_x.begin(), sx_x.end())-sx_x.begin()];
            y2 = sx_y[max_element(sx_x.begin(), sx_x.end())-sx_x.begin()];
          }

          //결국 위 함수는 벽면선분의 양 끝점 좌표를 정방향으로 정렬하는 역할이다. x2-x1값이 10보다 작은 경우는 y좌표가 증가하는 것을 정방향으로 삼고, 그 외에는 x좌표가 증가하는 것을 정방향으로 삼는다.

          Vec4i l1 = {x1,y1,x2,y2}; //
          vector<float> line_ref = lineAnalysis(l1);

          line(map, Point(x1, y1), Point(x2, y2), Scalar(255,255,0), 2);
          circle(map,Point(MAP_WIDTH/2+line_ref[1], MAP_HEIGHT/2+line_ref[2]),4, cv::Scalar(255,255,0), -1);


          if(pos_o >= 0){ //로봇 방향이 앞을 보고 있을 때
            sx_o = 2*atan(1) + line_ref[0]; //일단 sx_o는 90도 더하기 x1y1x2y2 선분의 경사각=
            if(pos_o < 2*atan(1) && line_ref[0] > 0){ //각도 0도에서 90도 사이이고 경사각이 양수일 때
              sx_o = 2*atan(1) - line_ref[0]; //
            }
            if(pos_o > 2*atan(1) && line_ref[0] < 0){
              sx_o = -2*atan(1) + line_ref[0];
            }
          }
          else{ //방향이 뒤를 보고 있을 때
            sx_o = -2*atan(1) + line_ref[0];
            if(pos_o > -2*atan(1) && line_ref[0] < 0){
              sx_o = -2*atan(1) - line_ref[0];
            }
            if(pos_o < -2*atan(1) && line_ref[0] > 0){
              sx_o = 2*atan(1) + line_ref[0];
            }
          }


          if(cosf(sx_o-pos_o)>0.75){
            sx_cos = cosf(sx_o);
            sx_sin = sinf(sx_o);
          }
          else{
            sx_o = 0;
          }

          sx_len = line_ref[4]; //선분의 길이
          sx = line_ref[3]; //선분까지의 수직발길이
        }








        float sxo = 0;
        float sxo_o = 0;
        float sxo_cos = 0;
        float sxo_sin = 0;
        float sxo_len = 0;
        if(sxo_x.size() > 1){
          int x1 = *min_element(sxo_x.begin(), sxo_x.end());
          int x2 = *max_element(sxo_x.begin(), sxo_x.end());
          int y1, y2;
          if(abs(x2-x1)<10){
            y1 = *min_element(sxo_y.begin(), sxo_y.end());
            y2 = *max_element(sxo_y.begin(), sxo_y.end());
            x1 = sxo_x[min_element(sxo_y.begin(), sxo_y.end())-sxo_y.begin()];
            x2 = sxo_x[max_element(sxo_y.begin(), sxo_y.end())-sxo_y.begin()];
          }
          else{
            y1 = sxo_y[min_element(sxo_x.begin(), sxo_x.end())-sxo_x.begin()];
            y2 = sxo_y[max_element(sxo_x.begin(), sxo_x.end())-sxo_x.begin()];
          }

          Vec4i l1 = {x1,y1,x2,y2};
          vector<float> line_ref = lineAnalysis(l1);

          line(map, Point(x1, y1), Point(x2, y2), Scalar(255,255,0), 2);
          circle(map,Point(MAP_WIDTH/2+line_ref[1], MAP_HEIGHT/2+line_ref[2]),4, cv::Scalar(255,255,0), -1);

          if(pos_o >= 0){
            sxo_o = 2*atan(1) + line_ref[0];
            if(pos_o < 2*atan(1) && line_ref[0] > 0){
              sxo_o = -2*atan(1) + line_ref[0];
            }
            if(pos_o > 2*atan(1) && line_ref[0] < 0){
              sxo_o = -2*atan(1) + line_ref[0];
            }
          }
          else{
            sxo_o = -2*atan(1) + line_ref[0];
            if(pos_o > -2*atan(1) && line_ref[0] < 0){
              sxo_o = 2*atan(1) + line_ref[0];
            }
            if(pos_o < -2*atan(1) && line_ref[0] > 0){
              sxo_o = 2*atan(1) + line_ref[0];
            }
          }
          if(cosf(sx_o-pos_o)>0.75){ //sxo_o 아님?
            sxo_cos = cosf(sxo_o);
            sxo_sin = sinf(sxo_o);
          }
          else{
            sxo_o = 0;
          }
          sxo_len = line_ref[4];
          sxo = line_ref[3];
        }

        float lx = 0;
        float lx_o = 0;
        float lx_cos = 0;
        float lx_sin = 0;
        float lx_len = 0;
        if(lx_x.size() > 1){
          int x1 = *min_element(lx_x.begin(), lx_x.end());
          int x2 = *max_element(lx_x.begin(), lx_x.end());
          int y1, y2;
          if(abs(x2-x1)<10){
            y1 = *min_element(lx_y.begin(), lx_y.end());
            y2 = *max_element(lx_y.begin(), lx_y.end());
            x1 = lx_x[min_element(lx_y.begin(), lx_y.end())-lx_y.begin()];
            x2 = lx_x[max_element(lx_y.begin(), lx_y.end())-lx_y.begin()];
          }
          else{
            y1 = lx_y[min_element(lx_x.begin(), lx_x.end())-lx_x.begin()];
            y2 = lx_y[max_element(lx_x.begin(), lx_x.end())-lx_x.begin()];
          }
          Vec4i l1 = {x1,y1,x2,y2};
          vector<float> line_ref = lineAnalysis(l1);

          line(map, Point(x1, y1), Point(x2, y2), Scalar(0,255,255), 2);
          circle(map,Point(MAP_WIDTH/2+line_ref[1], MAP_HEIGHT/2+line_ref[2]),4, cv::Scalar(0,255,255), -1);

          if(pos_o <= 2*atan(1) && pos_o >= -2*atan(1)){
            lx_o = line_ref[0];
          }
          else if(pos_o < -2*atan(1)){
            lx_o = -4*atan(1) + line_ref[0];
            if(line_ref[0] < 0){
              lx_o = line_ref[0];
            }
          }
          else if(pos_o > 2*atan(1)){
            lx_o = 4*atan(1) + line_ref[0];
            if(line_ref[0] > 0){
              lx_o = line_ref[0];
            }
          }
          if(cosf(sx_o-pos_o)>0.75){
            lx_cos = cosf(lx_o);
            lx_sin = sinf(lx_o);
          }
          else{
            lx_o = 0;
          }
          lx_len = line_ref[4];
          lx = line_ref[3];
        }

        float lxo = 0;
        float lxo_o = 0;
        float lxo_cos = 0;
        float lxo_sin = 0;
        float lxo_len = 0;

        if(lxo_x.size() > 1){
          vector<float> lxoh_x, lxoh_y, lxoe_x, lxoe_y;
          for(int i=0; i<lxo_perp.size(); i++){
            if(abs(lxo_perp[i]-*min_element(lxo_perp.begin(), lxo_perp.end()))*MAP_RESOL < 0.4){
              lxoh_x.push_back(lxo_x[i*2]);
              lxoh_x.push_back(lxo_x[i*2+1]);
              lxoh_y.push_back(lxo_y[i*2]);
              lxoh_y.push_back(lxo_y[i*2+1]);
            }
            else{
              lxoe_x.push_back(lxo_x[i*2]);
              lxoe_x.push_back(lxo_x[i*2+1]);
              lxoe_y.push_back(lxo_y[i*2]);
              lxoe_y.push_back(lxo_y[i*2+1]);
            }
          }

          if(lxo_x.size() > 1){
            int x1 = *min_element(lxoh_x.begin(), lxoh_x.end());
            int x2 = *max_element(lxoh_x.begin(), lxoh_x.end());
            int y1, y2;
            if(abs(x2-x1)<10){
              y1 = *min_element(lxoh_y.begin(), lxoh_y.end());
              y2 = *max_element(lxoh_y.begin(), lxoh_y.end());
              x1 = lxoh_x[min_element(lxoh_y.begin(), lxoh_y.end())-lxoh_y.begin()];
              x2 = lxoh_x[max_element(lxoh_y.begin(), lxoh_y.end())-lxoh_y.begin()];
            }
            else{
              y1 = lxoh_y[min_element(lxoh_x.begin(), lxoh_x.end())-lxoh_x.begin()];
              y2 = lxoh_y[max_element(lxoh_x.begin(), lxoh_x.end())-lxoh_x.begin()];
            }

            Vec4i l1 = {x1,y1,x2,y2};
            vector<float> line_ref = lineAnalysis(l1);

            line(map, Point(x1, y1), Point(x2, y2), Scalar(0,255,255), 2);
            circle(map,Point(MAP_WIDTH/2+line_ref[1], MAP_HEIGHT/2+line_ref[2]),4, cv::Scalar(0,255,255), -1);

            if(pos_o <= 2*atan(1) && pos_o >= -2*atan(1)){
              lxo_o = line_ref[0];
            }
            else if(pos_o < -2*atan(1)){
              lxo_o = -4*atan(1) + line_ref[0];
              if(line_ref[0] <= -atan(1)){
                lxo_o = line_ref[0];
              }
              if(line_ref[0] < 0 && line_ref[0] > -atan(1)){
                lxo_o = 4*atan(1) + line_ref[0];
              }
            }
            else if(pos_o > 2*atan(1)){
              lxo_o = 4*atan(1) + line_ref[0];
              if(line_ref[0] >= atan(1)){
                lxo_o = line_ref[0];
              }
              if(line_ref[0] > 0 && line_ref[0] < atan(1)){
                lxo_o = -4*atan(1) + line_ref[0];
              }
            }
            if(cosf(sx_o-pos_o)>0.75){
              lxo_cos = cosf(lxo_o);
              lxo_sin = sinf(lxo_o);
            }
            else{
              lxo_o = 0;
            }
            lxo_len = line_ref[4];
            lxo = line_ref[3];
          }
        }






        if(pos_x == 0 && pos_y == 0){ //이전에 설정한 적 없어서 무조건 이거 통과
          vector<float> cyl_pos;
          cyl_pos = cylinderOdometry(cyl_rel_x, cyl_rel_y);
          pos_o = cyl_pos[2];
          pos_x = 3.2;
          pos_y = 0.5;
        }
        else{
          if(lx_cos + lxo_cos + sx_cos + sxo_cos != 0){
            pos_o = atan2f(lx_sin + lxo_sin + sx_sin + sxo_sin, lx_cos + lxo_cos + sx_cos + sxo_cos);
          }

          if(lx_len < 30){
            pos_x = 3.0+lxo*MAP_RESOL;
          }
          else if(lxo_len < 30){
            pos_x = 8.0-lx*MAP_RESOL;
          }
          else{
            pos_x = (11.0 + lxo*MAP_RESOL - lx*MAP_RESOL)/2.0;
          }

          if(sx_len < 30){
            pos_y = sxo*MAP_RESOL;
          }
          else if(sxo_len < 30){
            pos_y = 3.0-sx*MAP_RESOL;
          }
          else{
            pos_y = (3.0 + sxo*MAP_RESOL - sx*MAP_RESOL)/2.0;
          }
        }

        if(init_odom == 0){
          pos_x = 0.0;
          pos_y = 0.0;
          pos_o = 0.0;
        }

      	odometry.x=pos_x;
      	odometry.y=pos_y;
      	odometry.z=pos_o;
      	pub.publish(odometry);

        ros::spinOnce();
    }

    return 0;
}
