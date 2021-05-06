#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;

int action;

int len;
int n;
int left_points,right_points;

#define ENTRANCE 1
#define BALLHARVESTING 2


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	map_mutex.lock();

	int count = scan->angle_max / scan->angle_increment;
    lidar_size=count;
    left_points=0; right_points=0;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];
        if (lidar_distance[i]< 0.7 && lidar_degree[i]<90 && lidar_degree[i]>0){
			left_points++;
        } else if ( lidar_distance[i] < 0.7 && lidar_degree[i]>270){
			right_points++;
        }
    }

	map_mutex.unlock();

}
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }

}

void control_entrance(geometry_msgs::Twist *targetVel)
{
	std::cout << "Entrance Zone Control" << std::endl;
	targetVel->linear.x  = 100;
	targetVel->angular.z = 0;
	map_mutex.lock();
	int threshold = 10;
	diff = left_points - right_points;
	map_mutex.unlock();
	
	if (diff < -threshold) { // control to leftside
		targetVel->linear.x  = 2;
		targetVel->angular.z = diff*0.1;  // TODO: change to PID control (Now P control)
	} else if (diff > threshold) { // control to rightside
		targetVel->linear.x  = 2;
		targetVel->angular.z = -diff*0.1;  // TODO: change to PID control (Now P control)
	} else { // Just move forward
		targetVel->linear.x  = 2;
		targetVel->angular.z = 0;
	}

}

void control_ballharvesting(geometry_msgs::Twist *targetVel)
{
	std::cout << "Ball Harvesting Control" << std::endl;
	targetVel->linear.x  = 100;
	targetVel->angular.z = 0;
	while (harversed < 5){

	}
}

int select_control_method(){
	if (true) return ENTRANCE;
	else return BALLHARVESTING;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
	ros::Publisher commandVel = n.advertise<geometry_msgs::Twist>("/command_vel", 10);

	// ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
	// ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);

	int control_method = ENTRANCE;

    while(ros::ok){
    	geometry_msgs::Twist targetVel;

    	if (control_method == ENTRANCE) {
    		control_entrance(&targetVel);
    	} else if (control_method == BALLHARVESTING) {
    		control_ballharvesting(&targetVel);
    	} else {
    		std::cout << "ERROR: NO CONTROL METHOD" << std::endl; // Unreachable statement
    	}
		commandVel.publish(targetVel);
		

		// std_msgs::Float64 left_wheel_msg;
		// std_msgs::Float64 right_wheel_msg;
		// left_wheel_msg.data=1;   // set left_wheel velocity
		// right_wheel_msg.data=1;  // set right_wheel velocity
		// pub_left_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
		// pub_right_wheel.publish(right_wheel_msg);  // publish right_wheel velocity

		// for(int i = 0; i < lidar_size; i++) {
		//     std::cout << "degree : " << lidar_degree[i] << "  distance : " << lidar_distance[i] << std::endl;
		// }
		// for(int i = 0; i < ball_number; i++) {
		// 	std::cout << "ball_X : " << ball_X[i] << "  ball_Y : " << ball_Y[i] << std::endl;
		// }


	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}
