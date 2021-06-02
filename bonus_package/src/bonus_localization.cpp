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


int cnt = 0;
float sum = 0;
int delivery_mode=1;
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


void lidar_cb(sensor_msgs::LaserScan msg){

    // angle in radian
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;

    // size of range vector
    int len = range.size();
    float angle_temp;

    /// 1. LaserScan msg to PCL::PointXYZ

    // initializae pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new::pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());

    new_cloud->is_dense = false;
    new_cloud->width = len;
    new_cloud->height = 1;
    new_cloud->points.resize(len);

    // fill the pointcloud
    for(int i = 0; i < len; i++){
        angle_temp =angle_min + angle_increment*i;
        if (std::isinf(range[i])==false){
            new_cloud->points[i].x = range[i] * cos(angle_temp);
            new_cloud->points[i].y = range[i] * sin(angle_temp);
            new_cloud->points[i].z = 1;
        }
        else {
            inf_points->indices.push_back(i);
        }
    }

    // Remove infinite distance points from new_cloud
    extract.setInputCloud(new_cloud);
    extract.setIndices(inf_points);
    extract.setNegative(true);
    extract.filter(*new_cloud);

    // 2. Get transformation between previous pointcloud and current pointcloud

    // transMtx_prev : transformation matrix at time (t-1)
    // transMtx_now : transformation matrix at time (t)
    // 4X4 transformation matrix (3X3: rotation matrix, 3X1: translation vector)

    if(prev_cloud->width == 0){

        // initialize transformation matrix. initial posiiton: x = 0, y = 0, theta = 0;
        transMtx_prev << cos(M_PI/2), -sin(M_PI/2), 0, 0,
                        sin(M_PI/2), cos(M_PI/2), 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
    }

    else{

        // ICP algorithm
        // https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html#iterative-closest-point
        // https://pcl.readthedocs.io/projects/tutorials/en/latest/interactive_icp.html#interactive-icp
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(prev_cloud);
        icp.setInputTarget(new_cloud);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        // std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        // icp.getFitnessScore() << std::endl;
        // std::cout << icp.getFinalTransformation() << std::endl;
        sum += icp.getFitnessScore();
        cnt++;
        transMtx_delta = icp.getFinalTransformation();

    // 3. Get current transformation matrix using previous transformation and ICP result
    //  (Odometry calculation)

        // TO DO START

        transMtx_now = transMtx_prev * transMtx_delta;

        // TO DO END

    // 4. Get current position from transformation matrix

        // TO DO START

        robot_pos.x = transMtx_now(0, 3)*100;
        robot_pos.y = transMtx_now(1,3)*100;
        if(transMtx_now(0,0)>=0 && transMtx_now(1,0)>=0){
            robot_pos.z=atan(transMtx_now(1,0)/transMtx_now(0,0));
        }else if(transMtx_now(0,0)<=0 && transMtx_now(1,0)>=0){
            robot_pos.z=M_PI+atan(transMtx_now(1,0)/transMtx_now(0,0));
        }else if(transMtx_now(0,0)<=0 && transMtx_now(1,0)<=0){
            robot_pos.z=M_PI+atan(transMtx_now(1,0)/transMtx_now(0,0));
        }else if(transMtx_now(0,0)>=0 && transMtx_now(1,0)<=0){
            robot_pos.z=2*M_PI+atan(transMtx_now(1,0)/transMtx_now(0,0));
        }

        // TO DO END

        transMtx_prev = transMtx_now; // Save current transformation matrix in transMtx_prev

    }
    // 5. Save new_cloud in prev_cloud

    prev_cloud = new_cloud;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "bonus_localization");
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
