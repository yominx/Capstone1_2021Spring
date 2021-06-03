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
#include "core_msgs/goal_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#define RAD2DEG(x) ((x)*180./M_PI)

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int left_points,right_points;
int prev_diff_points = 0;
int left_back_pts, right_back_pts;
int out_of_range_pts;

/* robot position variables */
float pos_x, pos_y, pos_o;
float target_x, target_y;
float target_o=0;
float diff_o;
float dist;

bool is_initial_step=true;

float target_distance=0;
float target_orientation=0;
float target_dist_prev=0;
float target_ori_prev=0;

int waytype;

//ball pickup&dumping part started
int delivery=0;
int delivery_count=0;
int ball_count=0;
//ball pickup&dumping part ended
ros::Publisher commandVel;
ros::Publisher zone;
ros::Publisher ball_number;
ros::Publisher ball_delivery;
geometry_msgs::Twist targetVel;

bool PASSED_STEP = false;
cv::Mat buffer_depth;

float v_linear;
float v_angular;
bool STOP_FLAG = false;

#define ENTRANCE 1
#define BALLHARVESTING 2
int control_method = ENTRANCE;

#define BALL 	1
#define PILLAR 	2
#define GOAL 	3
#define ROTATE 4

#define DEBUG_HARVEST true

using namespace std;

void control_harvest(geometry_msgs::Twist* targetVel);
void control_ballharvesting(geometry_msgs::Twist* targetVel);
// void control_entrance(geometry_msgs::Twist* targetVel);
void update_delivery_info();
void publish_delivery_info();

void position_Callback(const geometry_msgs::Vector3::ConstPtr& robot_pos) {
	pos_x = 50 + robot_pos->x;
	pos_y = 350 - robot_pos->y;
	pos_o = robot_pos->z;
}

int flag=0;
int COUNT=0;


int turn=0;

int initial_step=1;

int count2=0;

bool nosee= false;

void closest_Callback(const std_msgs::Float32MultiArray::ConstPtr& closest){
	



		target_distance = closest->data[0]*100;
		target_orientation = closest->data[1];
		initial_step++;

	if (target_distance>9000){
		nosee=true;
	}else{
		turn=0;
		nosee=false;
	}
	
	if(nosee || (initial_step>10 && fabs(target_distance-target_dist_prev)>35 && fabs(target_orientation-target_ori_prev)>0.1 ) ){
		target_distance = target_dist_prev;
		target_orientation = target_ori_prev;
	}





	if(target_distance>0 && target_distance < 60){
		
		flag=1;

	}
	
	cout<<"dist "<<target_distance<<" ori "<<target_orientation<<endl;

	if(~nosee){
		target_dist_prev=target_distance;
		target_ori_prev=target_orientation;
	}

	diff_o = target_orientation;
	dist = target_distance;


	control_ballharvesting(&targetVel);
	control_harvest(&targetVel);
	update_delivery_info();
	publish_delivery_info();
	commandVel.publish(targetVel);
}



float goal_distance=0;
float goal_orientation=0;

float goal_dist_prev=0;
float goal_ori_prev=0;

bool noseegoal;
int turn2=0;

float distg;
float diff_og;

int goalsize;

void goal_Callback(const core_msgs::goal_position::ConstPtr& goal_position){


  goalsize = goal_position->size;

  if (goalsize>=1){
  for(int i = 0; i < goalsize; i++)
  {
	goal_distance = goal_position->dist[0];
	goal_orientation = goal_position->angle[0];
  }
}



	initial_step++;

	if (goal_distance>9000){
		noseegoal=true;
		goal_distance = goal_dist_prev;
		goal_orientation = goal_ori_prev;
	}else{
		turn2=0;
		noseegoal=false;
	}
	


	if(~noseegoal){
		goal_dist_prev=goal_distance;
		goal_ori_prev=goal_orientation;
	}

	diff_og = goal_orientation;
	distg = goal_distance;
}











bool close_enough = false;

void control_ballharvesting(geometry_msgs::Twist *targetVel)
{
	float ANGLE_THRESHOLD = M_PI/80;

	float BALL_DIST_THRESHOLD = 42;
	float PILLAR_DIST_THRESHOLD = 50;
	float GOAL_DIST_THRESHOLD = 67;

	float angle_sign=diff_o/fabs(diff_o);
	float angle_signg=diff_og/fabs(diff_og);

	// cout << "Ball Harvesting Control" << endl;
	// cout << "[CONTROL] Angle difference is " << diff_o << endl;
	// cout << "DISTANCE IS " << dist << endl;
	// cout << "ANGLE DIFF IS " << diff_o << endl;

	cout<<"turn is "<<turn<<"nosee is "<<nosee<<" ball_count "<<ball_count<< " flag " <<flag<<" istrue "<< (fabs(diff_o) > ANGLE_THRESHOLD) <<endl;

	cout<<"diff_o is"<<diff_o<<endl;



		if (ball_count<3){
			waytype=BALL;
			
			if(nosee && turn==1){
				targetVel->linear.x  = 0;
				targetVel->angular.z = 1;
			}else if(flag) {
				targetVel->linear.x  = 2;
				COUNT++;
				cout<<"COUNT is                        "<<COUNT<<endl;
				if(COUNT>130){
					close_enough=true;
				}

			}else if (fabs(diff_o) > ANGLE_THRESHOLD) {
				/* in place rotation ->should be modified*/
				if (fabs(diff_o) < ANGLE_THRESHOLD*2) {
					targetVel->linear.x  = 0;
					targetVel->angular.z = angle_sign;
				} else {
					targetVel->linear.x  = 0;
					targetVel->angular.z =angle_sign;
				}

			} else{
				/* move forward ->should be modified*/
				if (dist > 150) {
					targetVel->linear.x  = 3;
					targetVel->angular.z = 0;
				} else {
					targetVel->linear.x  = dist/50;
					targetVel->angular.z = 0;
				}
			}

		}else{
		waytype=GOAL;

			if(noseegoal && turn2==1){
				targetVel->linear.x  = 0;
				targetVel->angular.z = 1;

			}else if (fabs(diff_og) > ANGLE_THRESHOLD) {
				/* in place rotation ->should be modified*/
				if (fabs(diff_og) < ANGLE_THRESHOLD*2) {
					targetVel->linear.x  = 0;
					targetVel->angular.z = angle_signg/2.5;
				} else {
					targetVel->linear.x  = 0;
					targetVel->angular.z = angle_signg*2/3;
				}


			}else {
				if (distg > GOAL_DIST_THRESHOLD*3) {
					targetVel->linear.x  = 3;
					targetVel->angular.z = 0;
				} else if (distg > GOAL_DIST_THRESHOLD) {
					targetVel->linear.x  = 0.5 + (distg/GOAL_DIST_THRESHOLD - 1);
				} else {
					if (!STOP_FLAG && ((targetVel->linear.x)+(targetVel->angular.z)) != 0) {
						targetVel->linear.x  = targetVel->linear.x/4;
						targetVel->angular.z = targetVel->angular.z/4;
						// STOP_FLAG = true;
					} else {
						targetVel->linear.x  = 0;
						targetVel->angular.z = 0;
					}
				}
			}

		}


	return;
}






void control_harvest(geometry_msgs::Twist* targetVel){
	// cout << "HARVEST TYPE: " << waytype << endl;
	float ANGLE_THRESHOLD = M_PI/100;
	float angle_sign = (diff_o > 0 ? 1 : -1);

	if (waytype==ROTATE){
		targetVel->linear.x=0;
		targetVel->angular.z=0;
	}
	else if(waytype==BALL){
		if(close_enough){
			cout << "CLOSE ENOUGH! HARVEST BALL!!!!!!!!!!!!!!!!" << endl;
			delivery=1;
			targetVel->linear.x=0;
			targetVel->angular.z=0;
		}
	}
	else if(waytype == GOAL) {
	// 	if(is_initial_step){
	// 		if (fabs(diff_o) < ANGLE_THRESHOLD){
	// 			targetVel->linear.x  = 0;
	// 			targetVel->angular.z = 0;
	// 		}
	// 		else if (fabs(diff_o) < ANGLE_THRESHOLD*30) {
	// 			targetVel->linear.x  = 0;
	// 			targetVel->angular.z = angle_sign*0.4;
	// 		} else {
	// 			targetVel->linear.x  = 0;
	// 			targetVel->angular.z = angle_sign*0.6;
	// 		}
	// 	}
		int goal_SIZE = 80;
		if(distg < goal_SIZE){
			targetVel->linear.x  = 0;
			float angle_sign = (diff_o > 0 ? 1 : -1);
			if (is_initial_step){
				is_initial_step = false;
				target_o = (pos_o<M_PI) ? pos_o+M_PI : pos_o-M_PI;
				targetVel->angular.z = angle_sign*0.6;
			}
			else {
				if (abs(pos_o-target_o) < ANGLE_THRESHOLD){
					delivery = 2;
					targetVel->linear.x = 0;
					targetVel->angular.z = 0;
				}
				else if(abs(pos_o-target_o) < ANGLE_THRESHOLD*30){
					targetVel->linear.x = 0;
					targetVel->angular.z = angle_sign*0.4;
				}
				else {
					targetVel->linear.x = 0;
					targetVel->angular.z = angle_sign*0.6;
				}
			}
		}
	}
}



// void control_entrance(geometry_msgs::Twist *targetVel)
// {
// 	//cout << "Entrance Zone Control" << endl;

// 	float threshold = 8, MIN_DIST_THRESHOLD = 0.05, RADIUS = 0.8;

// 	map_mutex.lock();
// 	left_points=0; right_points=0; left_back_pts=0; right_back_pts=0; out_of_range_pts=0;

// 	for (int i = 0; i < lidar_size; i++) {
//         if (MIN_DIST_THRESHOLD < lidar_distance[i] && lidar_distance[i]< RADIUS
//         	&& 0 < lidar_degree[i] && lidar_degree[i] < 90){
// 			right_points++;
//         } else if (MIN_DIST_THRESHOLD < lidar_distance[i] && lidar_distance[i] < RADIUS
//         			&& 90 < lidar_degree[i] && lidar_degree[i] < 180){
// 			left_points++;
//         } else if (-1< lidar_distance[i] && lidar_distance[i]< RADIUS
//         	&& -180 < lidar_degree[i] && lidar_degree[i] < -90){
// 			left_back_pts++;
// 		} else if (MIN_DIST_THRESHOLD < lidar_distance[i] && lidar_distance[i]< RADIUS
//         	&& -90 < lidar_degree[i] && lidar_degree[i] < 0){
// 			right_back_pts++;
// 		} else if (3< lidar_distance[i]
//         	&& 0 < lidar_degree[i] && lidar_degree[i] < 180){
// 			out_of_range_pts++;
// 		}

// 		// cout << "LEFT " << left_points << " RIGHT " << right_points << endl;
// 		cout <<" LB "<<left_back_pts<<" RB "<<right_back_pts<<endl;
// 		cout <<" OOR "<<out_of_range_pts<<endl;
// 		int diff = left_points - right_points;
// 		if (diff < -threshold) { // control to leftside
// 			targetVel->linear.x  = 4;
// 			targetVel->angular.z = -(diff+threshold)*0.04 + (diff-prev_diff_points)*2;  // TODO: change to PID control (Now P control)
// 		} else if (diff > threshold) { // control to rightside
// 			targetVel->linear.x  = 4;
// 			targetVel->angular.z = -(diff-threshold)*0.04 - (diff-prev_diff_points)*2;  // TODO: change to PID control (Now P control)
// 		} else { // Just move forward
// 			targetVel->linear.x  = 5;
// 			targetVel->angular.z = 0;
// 		}

// 		if (targetVel->angular.z > 1.7) targetVel->angular.z = 1.7;
// 		else if (targetVel->angular.z < -1.7) targetVel->angular.z = -1.7;

// 		prev_diff_points = diff;
// 	}
// 	targetVel->angular.x = -50;

// 	map_mutex.unlock();
// }


// void control_ballharvesting(geometry_msgs::Twist *targetVel)
// {
// 	float ANGLE_THRESHOLD = M_PI/100;
// 	float BALL_DIST_THRESHOLD = 42;
// 	float GOAL_DIST_THRESHOLD = 67;
// 	float angle_sign = (diff_o > 0 ? 1 : -1);
// 	cout<< "dist is "<< dist<< "diff_o is " << diff_o<<endl;
// 	switch (waytype) {
// 		case BALL:
// 			if (fabs(diff_o) < ANGLE_THRESHOLD){
// 				targetVel->linear.x  = 0;
// 				targetVel->angular.z = 0;
// 			}
// 			else if (fabs(diff_o) < ANGLE_THRESHOLD*2) {
// 					targetVel->linear.x  = 0;
// 					targetVel->angular.z = angle_sign*0.4;
// 				} else {
// 					targetVel->linear.x  = 0;
// 					targetVel->angular.z = angle_sign*0.6;
// 				}
// 			}
// 			if(dist <0){ 4
// 				cout << "CLOSE ENOUGH! HARVEST BALL" << endl;
// 				delivery=1;
// 				targetVel->linear.x=0;
// 				targetVel->angular.z=0;
// 			} else if (dist < 100){
// 				targetVel->linear.x  = dist/50;
// 				targetVel->angular.z = 0;
// 			} else {
// 				targetVel->linear.x  = 3;
// 				targetVel->angular.z = 0;
// 			}
//
//
// 			cout<< "x "<< targetVel->linear.x << "z " << targetVel->angular.z<<endl;
// 			break;
// 		case PILLAR:
// 			break;
// 		case GOAL:
// 			if (waytype==4){
// 				targetVel->linear.x  = 0;
// 				targetVel->angular.z = 1;
// 			}
// 			if(is_initial_step){
// 				if (fabs(diff_o) > ANGLE_THRESHOLD) {
// 					if (fabs(diff_o) < ANGLE_THRESHOLD*2) {
// 						targetVel->linear.x  = 0;
// 						targetVel->angular.z = angle_sign*0.4;
// 					} else {
// 						targetVel->linear.x  = 0;
// 						targetVel->angular.z = angle_sign*0.6;
// 					}
// 				}
// 			}
// 			int goal_SIZE = 80;
// 			if(dist < goal_SIZE){
// 				targetVel->linear.x  = 0;
// 				float angle_sign = (diff_o > 0 ? 1 : -1);
// 				if (is_initial_step){
// 					is_initial_step = false;
// 					target_o = (pos_o<M_PI) ? pos_o+M_PI : pos_o-M_PI;
// 					targetVel->angular.z = angle_sign*0.6;
// 				}
// 				else {
// 					if (abs(pos_o-target_o) < ANGLE_THRESHOLD){
// 						delivery = 2;
// 						targetVel->linear.x = 0;
// 						targetVel->angular.z = 0;
// 					}
// 					else if(abs(pos_o-target_o) < ANGLE_THRESHOLD*30){
// 						targetVel->linear.x = 0;
// 						targetVel->angular.z = angle_sign*0.4;
// 					}
// 					else {
// 						targetVel->linear.x = 0;
// 						targetVel->angular.z = angle_sign*0.6;
// 					}
// 				}
// 		}
// 		break;
// 		case ROTATE:
// 			targetVel->linear.x  = 0;
// 			targetVel->angular.z = 1;
// 	}
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bonus_data_integration");
    ros::NodeHandle n;

	// image_transport::ImageTransport it(n); //create image transport and connect it to node hnalder
	// image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depth_Callback);

    // ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    // ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
	ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("/closest_point", 1000, closest_Callback);
	ros::Subscriber sub_pos = n.subscribe<geometry_msgs::Vector3>("/robot_pos", 1000, position_Callback);
	ros::Subscriber sub_goal = n.subscribe<core_msgs::goal_position>("/goal_position", 1000, goal_Callback);



	// ros::Subscriber cur_vel = n.subscribe<std_msgs::Float32MultiArray>("/current_vel", 1000, vel_Callback); // for stop control

	commandVel = n.advertise<geometry_msgs::Twist>("/command_vel", 10);
	// zone = n.advertise<std_msgs::Int8>("/zone", 10);
	ball_number = n.advertise<std_msgs::Int8>("/ball_number", 10);
	ball_delivery = n.advertise<std_msgs::Int8>("/ball_delivery", 10);//pickup & dumping


	// double time_const_linear = 1; // to be modified with experiments
	// double time_const_angular = 1; // to be modified with experiments

    ros::Rate loop_rate(10);
	ros::spin();
    return 0;
}


void update_delivery_info(){
	if(delivery!=0){
		delivery_count++;
	}

	int th1=120;
	int th2=2000;
	cout<<"delivery_countt is            "<<delivery_count<<endl;

	if(delivery_count>th1 && delivery==1){
		delivery=0;
		delivery_count=0;
		cout<<"delivery_count="<<delivery_count<<endl;
		ball_count++;
		COUNT=0;
		close_enough=false;
		flag=0;
		turn=1;
		turn2=1;
		initial_step=1;


	}else if(delivery_count>th2 && delivery==2){
		delivery=0;
		delivery_count=0;
	}
}

void publish_delivery_info(){
	std_msgs::Int8 delivery_mode;
	delivery_mode.data=delivery;
	ball_delivery.publish(delivery_mode);

	std_msgs::Int8 ball_count_no;
	ball_count_no.data=ball_count;
	ball_number.publish(ball_count_no);
}
