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

#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "core_msgs/ball_position.h"
#include "core_msgs/goal_position.h"
#include "core_msgs/multiarray.h"


#define RAD2DEG(x) ((x)*180./M_PI)

/// MAP INFOS
/// Belows are written in pixel unit. 1 pixel = 1 cm.
/// MARGIN is similar to threshold.
#define ROBOT_SIZE 30 
#define PILLAR_RADIUS 10
#define	MARGIN 10

#define ROBOT 	1
#define BALL 	2
#define PILLAR 	3
#define GOAL 	4

boost::mutex map_mutex;
using namespace std;

ros::Publisher waypoints_publisher;


int robotX, 	robotY,
	ballX[10],  ballY[10], ball_exist[10],
	pillarX[5], pillarY[5],pillar_exist[5],
	goalX,		goalY;

int REMAINING_BALLS = 5;

class Node{
public:
	int x;
	int y;
	int type;
	int prev_node_idx;
	float dist_exact;
	float dist_h;
	bool confirmed;

	Node(int type_input, int x_input, int y_input){
		x 	= x_input;
		y 	= y_input;
		type = type_input;
		confirmed 	= false;
		prev_node_idx = -1;
		dist_exact 	= 10000;
		dist_h 		= 10000;
	}

	Node(){

	}
};

int Astar_plan(int size, int target_index, Node* node_list);

bool rotatable(int X, int Y){
	for(int i=0;i<10;i++){
		int diffX = ballX[i]-X, diffY = ballY[i]-Y;
		if(ball_exist[i] && diffX*diffX + diffY*diffY <= ROBOT_SIZE*ROBOT_SIZE)
			return false;
	}
	for(int i=0;i<5;i++){
		int diffX = pillarX[i]-X, diffY = pillarY[i]-Y;
		if(pillar_exist[i] && diffX*diffX + diffY*diffY <= ROBOT_SIZE*ROBOT_SIZE)
			return false;
	}
	// TODO: check for the walls, too.
	return true;
}


bool visible_robot(int ball_number){
	if(!ball_exist[ball_number]) return false;
	int x1 = ballX[ball_number], y1 = ballY[ball_number];
	int diffX = x1 - robotX, diffY = y1 - robotY;
	float a, b, num, den;
	
	if (diffX == 0){ // x+c=0
		for(int i=0;i<5;i++){
			if (pillar_exist[i] && abs(pillarX[i]-robotX) < (PILLAR_RADIUS + ROBOT_SIZE + MARGIN)) return false;
			if (ball_exist[i]   && abs(ballX[i]-robotX)   < (ROBOT_SIZE + MARGIN)) return false;
		}
		return true;
	} else { 
		// y=ax+b
		a = diffY/diffX; // slope
		b = y1-a*x1;
		den = sqrt(a*a+1);
		for(int i=0;i<5;i++){
			num = a*pillarX[i] + b - pillarY[i];
			if (pillar_exist[i] && num/den < (PILLAR_RADIUS + ROBOT_SIZE + MARGIN)) return false; // num/den == distance
		}
		for(int i=0;i<10;i++){
			if (i==ball_number) continue;

			num = a*pillarX[i] + b - pillarY[i];
			if (pillar_exist[i] && num/den < (ROBOT_SIZE + MARGIN)) return false; // num/den == distance
		}

		return true; 
	}
}

bool visible_arbitrary(int x1, int y1, int x2, int y2){
	int diffX = x1 - x2, diffY = y1 - y2;
	float a,b, num, den;
	
	if (diffX == 0){ // x+c=0
		for(int i=0;i<5;i++){
			if (pillar_exist[i] && abs(pillarX[i]-x1) < (PILLAR_RADIUS + ROBOT_SIZE + MARGIN)) return false;
			if (ball_exist[i]   && abs(ballX[i]-x1)   < (ROBOT_SIZE + MARGIN)) return false;
		}
		return true;
	} else { 
		// y=ax+b
		a = diffY/diffX; // slope
		b = y1-a*x1;
		den = sqrt(a*a+1);
		for(int i=0;i<5;i++){
			num = a*pillarX[i] + b - pillarY[i];
			if (pillar_exist[i] && num/den < (PILLAR_RADIUS + ROBOT_SIZE + MARGIN)) return false; // num/den == distance
		}
		for(int i=0;i<10;i++){
			num = a*pillarX[i] + b - pillarY[i];
			if (pillar_exist[i] && num/den < (ROBOT_SIZE + MARGIN)) return false; // num/den == distance
		}

		return true; 
	}
}


int get_shortest_index(int size, Node* node_list){ // '-1' means 'No balls are detected'
	// TODO: use A* to calculate more EXACT distance
	int index = -1;
	float min_dist, dist;

	for(int i=0;i<size;i++){
		Node node = node_list[i];

		if (node.type != BALL) continue;

		dist = pow(robotX - node.x, 2) + pow(robotY - node.y, 2);
		if(index == -1 || dist<min_dist){
			index = i;
			min_dist = dist;
		}
	}
	return index;
} 



void positions_callback(const core_msgs::multiarray::ConstPtr& object)
{
	Node nodes[20];
	int size = object->cols, node_number = 0;
	int GAP = ROBOT_SIZE + PILLAR_RADIUS + MARGIN;

	for (int i = 0; i < size; i++){ 
		int x = object->data[3*i+1];
		int y = object->data[3*i+2];

		if(object->data[3*i] == ROBOT || object->data[3*i] == BALL){
			nodes[node_number++] = Node(object->data[3*i], x, y); // type, x, y
		} else if (object->data[3*i] == PILLAR){
			nodes[node_number++] = Node(PILLAR, x + GAP, y + GAP);
			nodes[node_number++] = Node(PILLAR, x + GAP, y - GAP);
			nodes[node_number++] = Node(PILLAR, x - GAP, y + GAP);
			nodes[node_number++] = Node(PILLAR, x - GAP, y - GAP);
		}
	}

	while (REMAINING_BALLS > 0){
		int target_ball_index = get_shortest_index(node_number, nodes);
		if (target_ball_index == -1){ // No balls are found

			if(visible_arbitrary(robotX, robotY, robotX, robotY + 30)) {
				//return (robotX, robotY + 30);
			} else if (visible_arbitrary(robotX, robotY, robotX + 15, robotY + 15)) {
				//return (robotX + 15, robotY + 15);
			} else if (visible_arbitrary(robotX, robotY, robotX + 30, robotY)) {
				//return (robotX + 30, robotY);
			}

		} else { // make path_plan to nodes[i]
			// return Astar_plan(node_number, nodes); 
		}
	}



}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    ros::Subscriber sub_positions = n.subscribe<core_msgs::multiarray>("/position", 100, positions_callback);
	waypoints_publisher = n.advertise<core_msgs::multiarray>("/waypoints", 10);

    while(ros::ok){
	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}

int min_dist_idx(Node* node_list, std::vector<int>* visible_queue){
	float dist, min_dist, idx = -1;
	vector<int>::iterator it;

	for (it=visible_queue->begin(); it != visible_queue->end(); it++) { 
		Node cur_node = node_list[*it];
		if (cur_node.confirmed) continue;
		dist = cur_node.dist_exact + cur_node.dist_h;
	
		if(idx == -1 || dist < min_dist){
			min_dist = dist;
			idx = (*it);
		}
	}

	return idx;
}

void update_visibility(int cur_idx, int goal_index, int size, Node* node_list, std::vector<int>* visible_queue){
	int xt = node_list[goal_index].x, 	yt = node_list[goal_index].y;

	for(int i = 0; i < size; i++){
		if (node_list[i].confirmed) continue;
		if (visible_arbitrary(node_list[cur_idx].x, node_list[cur_idx].y, 
							  node_list[i].x,       node_list[i].y)){
			int diffX = node_list[cur_idx].x - node_list[i].x;
			int diffY = node_list[cur_idx].y - node_list[i].y;
			float distance = sqrt(pow(diffX, 2) + pow(diffY,2));

			int diffX_t = xt- node_list[i].x;
			int diffY_t = yt - node_list[i].y;
			float distance_t = sqrt(pow(diffX_t, 2) + pow(diffY_t,2));

			float dist_exact = node_list[cur_idx].dist_exact + distance;
			float dist_h = distance_t;
			if(dist_exact + dist_h < node_list[i].dist_exact + node_list[i].dist_h){
				node_list[i].dist_exact = dist_exact; 
				node_list[i].dist_h		= dist_h;
			}

			if(std::find(visible_queue->begin(), visible_queue->end(), i) == visible_queue->end()) {
				visible_queue->push_back(i);// queue doesn't contains i
			}
		}
	}
}

int Astar_plan(int size, int target_index, Node* node_list){ 
	// TODO: consider distance difference btw Collector & Lidar sensor when return
	std::vector<int> visible_queue;
	int cur_idx, min_idx, start_idx;
	for(int i = 0; i < size; i++){
		if (node_list[i].type == ROBOT) {
			cur_idx = i;
			start_idx = i;
			break;
		}
	}

	int xt = node_list[target_index].x, 	yt = node_list[target_index].y;
	int xcur = node_list[cur_idx].x, 		ycur = node_list[cur_idx].y;

	do{
		update_visibility(cur_idx, target_index, size, node_list, &visible_queue);
		cur_idx = min_dist_idx(node_list, &visible_queue);
		node_list[cur_idx].confirmed = true;
	} while (cur_idx != target_index); // if target is reachable, end of astar.

	Node cur_node = node_list[cur_idx];

	while (cur_node.prev_node_idx != start_idx) {
		cur_idx = cur_node.prev_node_idx;
		cur_node = node_list[cur_idx];
	}

	return cur_idx;
	// Compute visibility & iterate
}

