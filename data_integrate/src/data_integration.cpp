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

int left_back_pts, right_back_pts;
int out_of_range_pts;

/* robot position variables */
float pos_x;
float pos_y;
float pos_o;
float target_x;
float target_y;
float diff_o;
float dist;

#define ENTRANCE 1
#define BALLHARVESTING 2

using namespace std;

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	map_mutex.lock();
	cout << "LIDAR CALLBACK" << endl;
	int count = (scan->angle_max - scan->angle_min) / scan->angle_increment + 1;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];
        std::cout << "(deg, dist): "<< lidar_degree[i] << ", " << lidar_distance[i] << endl;

    }

	map_mutex.unlock();

}
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->angle[i];
        ball_Y[i] = position->dist[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }

}
void position_Callback(const geometry_msgs::Vector3::ConstPtr& robot_pos) {
	pos_x = robot_pos->x;
	pos_y = robot_pos->y;
	pos_o = robot_pos->z;
}
void target_Callback(const geometry_msgs::Vector3::ConstPtr& target_pos) {
	target_x = target_pos->x;
	target_y = target_pos->y;
	diff_o = atan2(target_y-pos_y, target_x-pos_x) - pos_o; // while -pos_o, |diff_o| may become > pi
	// atan2: -pi ~ pi, pos_o: 0 ~ 2pi => -3pi ~ pi
	if (diff_o < -M_PI) diff_o = diff_o + 2*M_PI;
	dist = sqrt(pow(target_y-pos_y, 2) + pow(target_x-pos_x, 2));
}


void control_entrance(geometry_msgs::Twist *targetVel)
{
	//cout << "Entrance Zone Control" << endl;
	
	float threshold = 8, MIN_DIST_THRESHOLD = 0.05, RADIUS = 0.8;

	map_mutex.lock();
	left_points=0; right_points=0; left_back_pts=0; right_back_pts=0; out_of_range_pts=0;
	for (int i = 0; i < lidar_size; i++) {
        if (MIN_DIST_THRESHOLD < lidar_distance[i] && lidar_distance[i]< RADIUS
        	&& 0 < lidar_degree[i] && lidar_degree[i] < 90){
			right_points++;
        } else if (MIN_DIST_THRESHOLD < lidar_distance[i] && lidar_distance[i] < RADIUS
        			&& 90 < lidar_degree[i] && lidar_degree[i] < 180){
			left_points++;
        } else if (-1< lidar_distance[i] && lidar_distance[i]< RADIUS
        	&& -180 < lidar_degree[i] && lidar_degree[i] < -90){
			left_back_pts++;
		} else if (MIN_DIST_THRESHOLD < lidar_distance[i] && lidar_distance[i]< RADIUS
        	&& -90 < lidar_degree[i] && lidar_degree[i] < 0){
			right_back_pts++;
		} else if (3< lidar_distance[i]
        	&& 0 < lidar_degree[i] && lidar_degree[i] < 180){
			out_of_range_pts++;
	}

	cout << "LEFT " << left_points << " RIGHT " << right_points << endl; 
	cout <<" LB "<<left_back_pts<<" RB "<<right_back_pts<<endl;
	cout <<" OOR "<<out_of_range_pts<<endl;
	int diff = left_points - right_points;
	if (diff < -threshold) { // control to leftside
		targetVel->linear.x  = 4;
		targetVel->angular.z = -(diff+threshold)*0.05;  // TODO: change to PID control (Now P control)
	} else if (diff > threshold) { // control to rightside
		targetVel->linear.x  = 4;
		targetVel->angular.z = -(diff-threshold)*0.05;  // TODO: change to PID control (Now P control)
	} else { // Just move forward
		targetVel->linear.x  = 4;
		targetVel->angular.z = 0;
	}
	}
	
	map_mutex.unlock();

}

void control_ballharvesting(geometry_msgs::Twist *targetVel)
{
	float ANGLE_THRESHOLD = M_PI/60;
	float DIST_THRESHOLD = 5;
	int angle_sign = (diff_o > 0 ? -1 : 1);

	cout << "Ball Harvesting Control" << endl;
	if (fabs(diff_o) > ANGLE_THRESHOLD) {
		/* in place rotation ->should be modified*/
		targetVel->linear.x  = 0;
		targetVel->angular.z = angle_sign*2;
	}
	else if (dist < DIST_THRESHOLD) {
		targetVel->linear.x  = 0;
		targetVel->angular.z = 0;
	}
	else {
		/* move forward ->should be modified*/
		targetVel->linear.x  = 4;
		targetVel->angular.z = 0;
	}
	return;
}

void in_place_turn() {
	return;
}
void move_forward() {
	return;
}
void stop() {
	return;
}

int select_control_method(){
	if (true) return ENTRANCE;
	else return BALLHARVESTING;
}


//ball pickup&dumping part started
int delivery=0;
int delivery_count=0;
int mode_input;
//ball pickup&dumping part ended

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    // ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    // for ballharvesting motor control
	ros::Subscriber sub_pos = n.subscribe<geometry_msgs::Vector3>("/robot_pos", 1000, position_Callback);
	ros::Subscriber sub_target = n.subscribe<geometry_msgs::Vector3>("/waypoint", 1000, target_Callback);
	
	ros::Publisher commandVel = n.advertise<geometry_msgs::Twist>("/command_vel", 10);
	ros::Publisher zone = n.advertise<std_msgs::Int8>("/zone", 10);
	ros::Publisher ball_delivery = n.advertise<std_msgs::Int8>("/ball_delivery", 10);//pickup & dumping

	// ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
	// ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);

	int control_method = ENTRANCE;
	double t;
	double time_const_linear = 1; // to be midified with experiments
	double time_const_angular = 1; // to be midified with experiments

    while(ros::ok){
    	geometry_msgs::Twist targetVel;

    	if (control_method == ENTRANCE) {
    		control_entrance(&targetVel);
    		commandVel.publish(targetVel);
    	}
    	else if (control_method == BALLHARVESTING) {
    		control_ballharvesting(&targetVel);
    		/*
    		if (targetVel->linear.x == 0) {
				t = (diff_o / targetVel->angular.z) * time_const_angular;
				ros::Time beginTime = ros::Time::now();
				ros::Duration delta_t = ros::Duration(t);
				ros::Time endTime = beginTime + delta_t;
				while(ros::Time::now() < endTime)
				{
					commandVel.publish(targetVel);
					ros::Duration(0.001).sleep();
				}
			}
			else {
				t = (dist / targetVel->linear.z) * time_const_linear;
				ros::Time beginTime = ros::Time::now();
				ros::Duration delta_t = ros::Duration(t);
				ros::Time endTime = beginTime + delta_t;
				while(ros::Time::now() < endTime)
				{
					commandVel.publish(targetVel);
					ros::Duration(0.001).sleep();
				}
			}
			*/
			commandVel.publish(targetVel);
    	}
    	else {
    		cout << "ERROR: NO CONTROL METHOD" << endl; // Unreachable statement
    	}


		std_msgs::Int8 zone_info;

		if( (0<left_back_pts && left_back_pts<11 && out_of_range_pts>5) || control_method== BALLHARVESTING){
			zone_info.data= BALLHARVESTING;
			control_method= BALLHARVESTING;
		}else{
			zone_info.data=ENTRANCE;
			control_method= ENTRANCE;
		}
		zone.publish(zone_info);


		// std_msgs::Float64 left_wheel_msg;
		// std_msgs::Float64 right_wheel_msg;
		// left_wheel_msg.data=1;   // set left_wheel velocity
		// right_wheel_msg.data=1;  // set right_wheel velocity
		// pub_left_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
		// pub_right_wheel.publish(right_wheel_msg);  // publish right_wheel velocity

		// for(int i = 0; i < lidar_size; i++) {
		//     cout << "degree : " << lidar_degree[i] << "  distance : " << lidar_distance[i] << endl;
		// }
		// for(int i = 0; i < ball_number; i++) {
		// 	cout << "ball_X : " << ball_X[i] << "  ball_Y : " << ball_Y[i] << endl;
		// }
	    
		//Ball pickup/dumping part started
		delivery=1;
			
		if(delivery!=0){
			delivery_count++;
		}

		int th1=1000;
		int th2=1000;
		if(delivery_count>th1 && delivery==1){
			delivery=0;
			delivery_count=0;
		}else if(delivery_count>th2 && delivery==2){
			delivery=0;
			delivery_count=0;
		}

		std_msgs::Int8 delivery_mode;
		delivery_mode.data=delivery;
		ball_delivery.publish(delivery_mode);
		//Ball pickup/dumping part ended

		    
		ros::Duration(0.025).sleep();
		ros::spinOnce();
    }

    return 0;
}
