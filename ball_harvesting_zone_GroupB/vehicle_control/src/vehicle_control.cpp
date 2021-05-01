#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "opencv2/opencv.hpp"
using namespace std;

float point_x[100];
float point_y[100];
float current_angle;
float end_angle;

int number;
void control_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

	number=msg->poses.size();

	cout<<number<<endl;

	for(int i=0; i<number; i++){
		point_x[i]=msg->poses[i].position.x;
		point_y[i]=msg->poses[i].position.y;
		cout<<point_x[i]<<" , "<<point_y[i]<<" , "<<endl;
	}
	current_angle=msg->poses[0].position.z;
	end_angle=msg->poses[number-1].position.z;
	//cout<<current_angle<<" , "<<end_angle<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vehicle_control_node");
	ros::NodeHandle n;

	ros::Subscriber sub_checkpoint = n.subscribe<geometry_msgs::PoseArray>("/checkpoint", 1000, control_Callback);
  //  ros::Subscriber sub_odometry = n.subscribe<geometry_msgs::Vector3>("/odometry", 1000, odometry_Callback);

	ros::Publisher pub_left_wheel = n.advertise<std_msgs::Float64>("/skeleton/left_wheel_velocity_controller/command", 10);
  ros::Publisher pub_right_wheel = n.advertise<std_msgs::Float64>("/skeleton/right_wheel_velocity_controller/command", 10);
  ros::Publisher pub_result = n.advertise<std_msgs::Int8>("/arrival", 1);

	std_msgs::Float64 left_wheel_msg;
	std_msgs::Float64 right_wheel_msg;
	std_msgs::Int8 result;

	double delta;
	double w=3;
	double d=0.2955;
	double r=0.075;
	double goal_angle;
	double t;
	
	result.data=1;
	pub_result.publish(result);

	ros::Rate loop_rate(10);

	while(ros::ok()){
		ros::spinOnce();
		
		for(int i=1; i<number; i++){
			if(point_x[i]==-1 && point_y[i]==-1) break;
			goal_angle=atan2(point_y[i]-point_y[i-1], point_x[i]-point_x[i-1]);
			cout<<"current"<<current_angle<<","<<"goal"<<goal_angle<<endl;

			delta=current_angle-goal_angle;
			cout<<delta<<"rad"<<endl;
			if(delta<0) delta=delta+atan(1)*8;
			if(delta>atan(1)*4){
				left_wheel_msg.data = -w;
				right_wheel_msg.data = w;
				t=d*(atan(1)*8-delta)/(2*w*r);
				ros::Time beginTime=ros::Time::now();
				ros::Duration delta_t = ros::Duration(t);
				ros::Time endTime=beginTime + delta_t;
				while(ros::Time::now()<endTime)
				{
					pub_left_wheel.publish(left_wheel_msg);
					pub_right_wheel.publish(right_wheel_msg);
					ros::Duration(0.001).sleep();
				}

			}
			else{
				left_wheel_msg.data = w;
				right_wheel_msg.data = -w;
				t=d*delta/(2*w*r);
				ros::Time beginTime=ros::Time::now();
				ros::Duration delta_t = ros::Duration(t);
				ros::Time endTime=beginTime + delta_t;
				while(ros::Time::now()<endTime)
				{
					pub_left_wheel.publish(left_wheel_msg);
					pub_right_wheel.publish(right_wheel_msg);
					ros::Duration(0.001).sleep();
				}
			}
			left_wheel_msg.data = 0; right_wheel_msg.data = 0;
			pub_left_wheel.publish(left_wheel_msg);
			pub_right_wheel.publish(right_wheel_msg);


			left_wheel_msg.data = w;
			right_wheel_msg.data = w;
			t=sqrt(pow(point_y[i]-point_y[i-1], 2)+pow(point_x[i]-point_x[i-1], 2))/(w*r);

			ros::Time beginTime=ros::Time::now();
			ros::Duration delta_t = ros::Duration(t);
			ros::Time endTime=beginTime + delta_t;

			while(ros::Time::now()<endTime)
			{
				pub_left_wheel.publish(left_wheel_msg);
				pub_right_wheel.publish(right_wheel_msg);
				ros::Duration(0.001).sleep();
			}
			left_wheel_msg.data = 0; right_wheel_msg.data = 0;
			pub_left_wheel.publish(left_wheel_msg);
			pub_right_wheel.publish(right_wheel_msg);

			current_angle=goal_angle;
			result.data=0;
			pub_result.publish(result);
		}

		goal_angle=end_angle;
		delta=current_angle-goal_angle;
		if(delta<0) delta=delta+atan(1)*8;
		if(delta>atan(1)*4){
			left_wheel_msg.data = -w;
			right_wheel_msg.data = w;
			t=d*(atan(1)*8-delta)/(2*w*r);
			ros::Time beginTime=ros::Time::now();
			ros::Duration delta_t = ros::Duration(t);
			ros::Time endTime=beginTime + delta_t;
			while(ros::Time::now()<endTime)
			{
				pub_left_wheel.publish(left_wheel_msg);
				pub_right_wheel.publish(right_wheel_msg);
				ros::Duration(0.001).sleep();
		}
		}

		else{
			left_wheel_msg.data = w;
			right_wheel_msg.data = -w;
			t=d*delta/(2*w*r);
			ros::Time beginTime=ros::Time::now();
			ros::Duration delta_t = ros::Duration(t);
			ros::Time endTime=beginTime + delta_t;
			while(ros::Time::now()<endTime)
			{
				pub_left_wheel.publish(left_wheel_msg);
				pub_right_wheel.publish(right_wheel_msg);
				ros::Duration(0.001).sleep();
			}
			left_wheel_msg.data = 0; right_wheel_msg.data = 0;
			pub_left_wheel.publish(left_wheel_msg);
			pub_right_wheel.publish(right_wheel_msg);
		}

		left_wheel_msg.data = 0; right_wheel_msg.data = 0;
		pub_left_wheel.publish(left_wheel_msg);
		pub_right_wheel.publish(right_wheel_msg);
		result.data=1;
		pub_result.publish(result);
		loop_rate.sleep();
	}
	return 0;
}
