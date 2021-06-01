#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>
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


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include "std_msgs/Int8.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

using namespace std;



int delivery_mode=0;
void delivery_mode_Callback(const std_msgs::Int8::ConstPtr& delivery){
  delivery_mode=delivery->data;
}


geometry_msgs::Vector3 robot_pos;

int lidar_size;
float lidar_degree[400]; 
float lidar_distance[400];

boost::mutex map_mutex;

pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new::pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud (new::pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f transMtx_now; // 초기 위치 대비 바로 전 과거 시점의 로봇의 위치를 나타내는 Transformation matrix
Eigen::Matrix4f transMtx_prev; // 초기 위치 대비 바로 전 현재 시점의 로봇의 위치를 나타내는 Transformation matrix
Eigen::Matrix4f transMtx_delta; //LiDAR 데이터를 이용한 ICP Dead Reckoning의 결과로 나오는 변화량을 나타내는 Transformation matrix


int first_step=1;
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan){

    if (delivery_mode !=0){
        map_mutex.lock();

        int i;
        lidar_size = (scan->angle_max - scan->angle_min)/ scan->angle_increment+1;
        for(i = 0; i < lidar_size; i++)
        {
            lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
            lidar_distance[i]=scan->ranges[i];
        }


        vector<float> wall_distance, wall_degree;

            int view_angle=0;
            float sum=0;
            float obstacle_avg_distance;
            float obstacle_width;

            if(lidar_distance[i] >0 && (std::isinf(lidar_distance[i])==false) ){ 
                wall_distance.push_back(lidar_distance[0]);
                wall_degree.push_back(lidar_degree[0]);
            }

            sum=sum+lidar_distance[0];

            for(int i = 1; i < lidar_size; i++)
            {
            if(lidar_distance[i] >0 && (std::isinf(lidar_distance[i])==false) ){ //smaller than the maximum liDAR range(distance)
                wall_distance.push_back(lidar_distance[i]);
                wall_degree.push_back(lidar_degree[i]);
                view_angle++;
                sum=sum+lidar_distance[i];
            }

            if(abs(lidar_distance[i] - lidar_distance[i-1]) > 0.3){
            //distance changed abruptly -> outermost wall or an another distant obstacle is detected.
                view_angle=view_angle-1;
                sum=sum-lidar_distance[i];
                obstacle_avg_distance=sum/(view_angle+1);
                obstacle_width=obstacle_avg_distance*(M_PI/180)*view_angle;
    //Debugging:cout<<obstacle_width<<"/"<<view_angle<<endl;

                if(0.07<obstacle_width && obstacle_width < 0.15 && view_angle>0 && abs(lidar_distance[i-1]-obstacle_avg_distance)<0.2 ){//threshold should be larger than the maximum width of the obstacle
                //this means that formerly detected object has small width, which means it is likely to be an obstacle

                for(int j=0; j<view_angle+2; j++){
                    wall_distance.pop_back();
                    wall_degree.pop_back();
                } //remove the added lidar data as much as the view angle of the obstacle
                wall_distance.push_back(lidar_distance[i]);
                wall_degree.push_back(lidar_degree[i]);//This turn's LiDAR data is measuring new

                }
                view_angle=0; //Initiallization
                sum=lidar_distance[i];
            }
            }



        // initializae pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new::pcl::PointCloud<pcl::PointXYZ>);

        new_cloud->is_dense = false;
        new_cloud->width = wall_distance.size();
        new_cloud->height = 1;
        new_cloud->points.resize(wall_distance.size());
        
        for(int i = 0; i < wall_distance.size(); i++){

                new_cloud->points[i].x = wall_distance[i]*cos(wall_degree[i]);
                new_cloud->points[i].y = -wall_distance[i]*sin(wall_degree[i]);
                new_cloud->points[i].z = 0; 
        }

        // 2. Get transformation between previous pointcloud and current pointcloud

        // transMtx_prev : transformation matrix at time (t-1)
        // transMtx_now : transformation matrix at time (t)
        // 4X4 transformation matrix (3X3: rotation matrix, 3X1: translation vector)

        if(prev_cloud->width == 0){
            // initialize transformation matrix. initial posiiton: x = 0, y = 0, theta = 0;
            transMtx_prev << cos(0), -sin(0), 0, 0,
                            sin(0), cos(0), 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1;
        }

        else{

            // ICP algorithm
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(initial_cloud);
            icp.setInputTarget(new_cloud);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);
    
            transMtx_now = icp.getFinalTransformation();

        // 3. Get current transformation matrix using previous transformation and ICP result
        //  (Odometry calculation)
            //transMtx_now =transMtx_prev*transMtx_delta;

        // 4. Get current position from transformation matrix
            


            robot_pos.x = transMtx_now(0, 3)*100;
            robot_pos.y = transMtx_now(1,3)*100;
            if(transMtx_now(0,0)>=0){
                robot_pos.z=atan(transMtx_now(1,0)/transMtx_now(0,0));
            }else if(transMtx_now(0,0)<0){
                robot_pos.z=6*atan(1)+atan(transMtx_now(1,0)/transMtx_now(0,0));
            }

            transMtx_prev = transMtx_now; // Save current transformation matrix in transMtx_prev

        }
        // 5. Save new_cloud in prev_cloud

        prev_cloud = new_cloud;

        if(first_step==1){
            initial_cloud=new_cloud;
            first_step=first_step-1;
        }
        map_mutex.unlock();
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_homework_node");
    ros::NodeHandle nh;
    ros::Subscriber delivery_mode_info = nh.subscribe<std_msgs::Int8>("/ball_delivery", 10, delivery_mode_Callback);
    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);

    ros::Publisher robot_pos_pub= nh.advertise<geometry_msgs::Vector3>("/robot_pos", 1); //odometry, 즉 robot의 위치를 Vector3로 발행한다.


    prev_cloud->width = 0;


    while(ros::ok()){


        ROS_INFO("pos : x = %f | y = %f | theta = %f", robot_pos.x, robot_pos.y, robot_pos.z);

        robot_pos_pub.publish(robot_pos);
        ros::spinOnce();
    }

    return 0;
}
