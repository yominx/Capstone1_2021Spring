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
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int8.h"


#define RAD2DEG(x) ((x)*180./M_PI)

/// MAP INFOS
/// Belows are written in pixel unit. 1 pixel = 1 cm.
/// MARGIN is similar to threshold.
#define ROBOT_SIZE 20 
#define PILLAR_RADIUS 10
#define	MARGIN 7
#define	THRESHOLD 7

#define ROBOT 	0
#define BALL 	1
#define PILLAR 	2
#define GOAL 	3

#define MAP_WIDTH 	600
#define MAP_HEIGHT 	400


boost::mutex node_lock;
using namespace std;
using namespace cv;

ros::Publisher waypoints_publisher;
Mat missionmap;

int robotX, 	robotY,
	ballX[10],  ballY[10], ballCount,
	pillarX[5], pillarY[5],pillarCount,
	goalX,		goalY;

int REMAINING_BALLS = 5;
bool END = false;

class NodeMap{
public:
	int x;
	int y;
	int type;
	int prev_node_idx;
	float dist_exact;
	float dist_h;
	bool confirmed;

	NodeMap(int type_input, int x_input, int y_input){
		x 	= x_input;
		y 	= y_input;
		type = type_input;
		confirmed 	= false;
		prev_node_idx = -1;
		dist_exact 	= 10000;
		dist_h 		= 10000;
	}

	NodeMap(){

	}
};

NodeMap nodes[40];


int Astar_plan(int size, int target_index, NodeMap* node_list);
void visualize(int size, NodeMap* nodes, int goal_index, int targetX, int targetY);


void publish_wayp(int x, int y, int z){
	geometry_msgs::Vector3 waypoint;
	cout << "Move to (" << x << ", " << y << ") for type " << z << endl;
	waypoint.x = x;
	waypoint.y = 400 - y;
	waypoint.z = z;
	waypoints_publisher.publish(waypoint);
}

// bool rotatable(int X, int Y){
// 	for(int i=0;i<10;i++){
// 		int diffX = ballX[i]-X, diffY = ballY[i]-Y;
// 		if(ball_exist[i] && diffX*diffX + diffY*diffY <= ROBOT_SIZE*ROBOT_SIZE)
// 			return false;
// 	}
// 	for(int i=0;i<5;i++){
// 		int diffX = pillarX[i]-X, diffY = pillarY[i]-Y;
// 		if(pillar_exist[i] && diffX*diffX + diffY*diffY <= ROBOT_SIZE*ROBOT_SIZE)
// 			return false;
// 	}
// 	// TODO: check for the walls, too.
// 	return true;
// }
bool visible_arbitrary(int x1, int y1, int x2, int y2){
	int diffX = x2 - x1, diffY = y2 - y1, ITER = 100;
	float stepX = diffX/ITER, stepY = diffY/ITER;
	float posX, posY, distsq;
	for(int i=0; i<ITER; i++){ // check collision for 100 steps (discrete)
		posX = x1+i*stepX;
		posY = y1+i*stepY;
		for(int i=0;i < pillarCount;i++){ // check pillar-collision
			distsq = pow(posX-pillarX[i], 2) + pow(posY-pillarY[i],2);
			if(distsq < pow(PILLAR_RADIUS+ROBOT_SIZE, 2)) return false;
		}
		for(int i=0;i<ballCount;i++){ // check ball-collision
			if((ballX[i] == x1 && ballY[i] == y1) 
				|| (ballX[i] == x2 && ballY[i] == y2))
					continue;
			distsq = pow(posX-ballX[i], 2) + pow(posY-ballY[i],2);
			if(distsq < pow(MARGIN+ROBOT_SIZE,2)) return false;
		}
	}
	return true;
}


// bool visible_arbitrary(int x1, int y1, int x2, int y2){
// 	int diffX = x1 - x2, diffY = y1 - y2;
// 	float a,b, num, den;
	
// 	if (diffX == 0){ // x+c=0
// 		for(int i=0;i<pillarCount;i++){
// 			if (abs(pillarX[i]-x1) < (PILLAR_RADIUS + ROBOT_SIZE + MARGIN)) return false;
// 		}
// 		for(int i=0;i<ballCount;i++){
// 			if (abs(ballX[i]-x1)   < (ROBOT_SIZE + MARGIN)) return false;
// 		}
// 		return true;
// 	} else { 
// 		// y=ax+b
// 		a = diffY/diffX; // slope
// 		b = y1-a*x1;
// 		den = sqrt(a*a+1);
// 		for(int i=0;i < pillarCount;i++){ // check pillar-collision
// 			num = a*pillarX[i] + b - pillarY[i];
// 			cout << "distance for "<<i<< "th pillar is " << abs(num/den) <<endl;
// 			if (abs(num/den) < (PILLAR_RADIUS + ROBOT_SIZE)) return false; // num/den == distance
// 		}
// 		for(int i=0;i<ballCount;i++){ // check ball-collision
// 			num = a*ballX[i] + b - ballY[i];
// 			cout << "distance for "<<i<< "th ball is " << abs(num/den) <<endl;
// 			if (abs(num/den) < (ROBOT_SIZE + MARGIN)) return false; // num/den == distance. ignore radius of the ball.
// 		}

// 		return true; 
// 	}
// }


int get_shortest_index(int size, NodeMap* node_list){ // '-1' means 'No balls are detected'
	// TODO: use A* to calculate more EXACT distance
	int index = -1;
	float min_dist, dist;

	for(int i=0;i<size;i++){
		NodeMap node = node_list[i];

		if (node.type != BALL) continue;

		dist = pow(robotX - node.x, 2) + pow(robotY - node.y, 2);
		if(index == -1 || dist<min_dist){
			index = i;
			min_dist = dist;
		}
	}
	return index;
} 

int buildMap(int size, NodeMap* nodes, const core_msgs::multiarray::ConstPtr& object){
	int node_number = 0;
	int GAP = ROBOT_SIZE + PILLAR_RADIUS + MARGIN;
	// reset prev data
	pillarCount = 0; ballCount = 0;

	for (int i = 0; i < size; i++){ 
		int x = object->data[3*i+1];
		int y = object->data[3*i+2];

		switch (object->data[3*i]){
			case ROBOT:
				cout << "[MAP] ROBOT POS: " << x << ", " << y << endl;
				nodes[node_number++] = NodeMap(ROBOT, x, y);
				robotX = x;
				robotY = y;
				break;
			case BALL:
				cout << "[MAP] BALL POS: " << x << ", " << y << endl;
				nodes[node_number++] = NodeMap(BALL, x, y);
				ballX[ballCount] = x;
				ballY[ballCount] = y;
				ballCount++;
				break;
			case PILLAR:
				cout << "[MAP] PILLAR POS: " << x << ", " << y << endl;
				nodes[node_number++] = NodeMap(PILLAR, x + GAP, y + GAP);
				nodes[node_number++] = NodeMap(PILLAR, x + GAP, y - GAP);
				nodes[node_number++] = NodeMap(PILLAR, x - GAP, y + GAP);
				nodes[node_number++] = NodeMap(PILLAR, x - GAP, y - GAP);
				pillarX[pillarCount] = x;
				pillarY[pillarCount] = y;
				pillarCount++;
				break;
			case GOAL:
				cout << "[MAP] GOAL POS: " << x << ", " << y << endl;
				nodes[node_number++] = NodeMap(GOAL, x, y);
				goalX = x;
				goalY = y;
				break;		
		}
	}
	return node_number;
}


void unknown_map_control(int node_number){
	cout << "Robot Position:" << robotX << ", " << robotY << endl;
	if(visible_arbitrary(robotX, robotY, robotX, robotY + 30)) {
		publish_wayp(robotX + 30, robotY, -1);
		visualize(node_number, nodes, -1, robotX + 30, robotY);
	} else if (visible_arbitrary(robotX, robotY, robotX + 15, robotY + 15)) {
		publish_wayp(robotX + 15, robotY + 15, -1);
		visualize(node_number, nodes, -1, robotX + 15, robotY + 15);
	} else if (visible_arbitrary(robotX, robotY, robotX + 30, robotY)) {
		publish_wayp(robotX, robotY + 30, -1);
		visualize(node_number, nodes, -1, robotX, robotY + 30);
	} else{
		cout << "ERROR: No points to move" << endl;
	}
}

void ballharvest_control(int node_number, int target_ball_index, NodeMap* nodes){
	int next_index = Astar_plan(node_number, target_ball_index, nodes); 
	if (next_index == -1){
		cout << "????????????????" << endl;
		unknown_map_control(node_number);		
	}
	int nextX = nodes[next_index].x;
	int nextY = nodes[next_index].y;

	if (pow(nextX-robotX,2) + pow(nextY-robotY,2) < pow(THRESHOLD,2)){ // close enough?
		NodeMap cur_node = nodes[target_ball_index];
		int cur_idx = target_ball_index;
		while (cur_node.prev_node_idx != next_index) {
			cur_idx = cur_node.prev_node_idx;
			cur_node = nodes[cur_idx];
		}

		visualize(node_number, nodes, target_ball_index, cur_node.x, cur_node.y);
		publish_wayp(cur_node.x, cur_node.y, cur_node.type);

	} else {
		visualize(node_number, nodes, target_ball_index, nextX, nextY);
		publish_wayp(nextX, nextY, nodes[next_index].type);
	}

}

void goal_control(int size, NodeMap* nodes){
	int goal_index = -1;
	for(int i=0; i<size; i++){
		NodeMap node = nodes[i];
		if (node.type == GOAL) break;
	}

	if(goal_index == -1){ // GOAL not found...
		cout << "[GOAL CONTROL] ERROR: GOAL IS NOT FOUND..." << endl;
	} else if(pow(nodes[goal_index].x-robotX,2) + pow(nodes[goal_index].y-robotY,2) < pow(THRESHOLD,2)){ // close enough
		END = true;
	} else {
		int next_index = Astar_plan(size, goal_index, nodes); 
		int nextX = nodes[next_index].x;
		int nextY = nodes[next_index].y;

		if (pow(nextX-robotX,2) + pow(nextY-robotY,2) < pow(THRESHOLD,2)){ // close enough?
			NodeMap cur_node = nodes[goal_index];
			int cur_idx = goal_index;
			while (cur_node.prev_node_idx != next_index) {
				cur_idx = cur_node.prev_node_idx;
				cur_node = nodes[cur_idx];
			}
			publish_wayp(cur_node.x, cur_node.y, cur_node.type);
		} else {
			publish_wayp(nextX, nextY, nodes[next_index].type);
		}
	}

}

void positions_callback(const core_msgs::multiarray::ConstPtr& object)
{
	if(END){
		publish_wayp(-1,-1,-1);
	}
	int size = (object->data.size())/3;
	int node_number = 0;

	cout << "[Callback] Position callback: " << size << " elements known" << endl;
	node_lock.lock();
	node_number = buildMap(size, nodes, object);
	cout << "[Mapping] Mapping complete" << endl;
	cout << "	PILLAR NUMBER: " << pillarCount << endl;
	cout << "	BALL   NUMBER: " << ballCount   << endl;
	// visualize(size, nodes, -1, 0, 0);

	if (REMAINING_BALLS > 0){
		int target_ball_index = get_shortest_index(node_number, nodes);
		if (target_ball_index == -1){ // No balls are found
			cout << "No balls are detected..." << endl;
			unknown_map_control(node_number);
		} else { // make path_plan to nodes[i]
			ballharvest_control(node_number, target_ball_index, nodes);
		}
	} else { // Go to goal point
		goal_control(node_number, nodes);
	}

	node_lock.unlock();
}

void ballcount_callback(const std_msgs::Int8::ConstPtr& count){
	REMAINING_BALLS = 5 - count->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;

    ros::Subscriber sub_positions = n.subscribe<core_msgs::multiarray>("/position", 100, positions_callback);
    ros::Subscriber sub_harvested_balls = n.subscribe<std_msgs::Int8> ("/ball_count", 100, ballcount_callback);
	waypoints_publisher = n.advertise<geometry_msgs::Vector3>("/waypoint", 10);

    while(ros::ok){
	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}

int min_dist_idx(NodeMap* node_list, std::vector<int>* visible_queue){
	float dist, min_dist, idx = -1;
	vector<int>::iterator it;

	for (it=visible_queue->begin(); it != visible_queue->end(); it++) { 
		NodeMap cur_node = node_list[*it];
		if (cur_node.confirmed) continue;
		dist = cur_node.dist_exact + cur_node.dist_h;
	
		if(idx == -1 || dist < min_dist){
			min_dist = dist;
			idx = (*it);
		}
	}

	return idx;
}

void update_visibility(int cur_idx, int goal_index, int size, NodeMap* node_list, std::vector<int>* visible_queue){
	int xt = node_list[goal_index].x, 	yt = node_list[goal_index].y;

	// cout << "Update visibility" << endl;
	for(int i = 0; i < size; i++){
		if (i == cur_idx || node_list[i].confirmed) continue;
		if (visible_arbitrary(node_list[cur_idx].x, node_list[cur_idx].y, 
							  node_list[i].x,       node_list[i].y)){
			// cout << i << "th object is visible from " << cur_idx << "th object." << endl;

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
				node_list[i].prev_node_idx = cur_idx;
			}

			// cout << "contains?" << endl;
			if(std::find(visible_queue->begin(), visible_queue->end(), i) == visible_queue->end()) {
				visible_queue->push_back(i);// queue doesn't contains i
			}
			// cout << "visibility done." << endl;
		}
	}
}


int Astar_plan(int size, int target_index, NodeMap* node_list){ 
	cout << "[Astar] perform Astar" << endl;
	std::vector<int> visible_queue;
	visible_queue.clear();
	int cur_idx = -1, min_idx, start_idx;
	for(int i = 0; i < size; i++){
		if (node_list[i].type == ROBOT) {
			cur_idx = i;
			start_idx = i;
			break;
		}
	}

	cout << "start index: " << cur_idx << endl << "target index: " << target_index << endl;
	cout << "Size is " << size << endl;
	int xt = node_list[target_index].x, 	yt = node_list[target_index].y;
	int xcur = node_list[cur_idx].x, 		ycur = node_list[cur_idx].y;

	do{
		if(cur_idx == -1){
			cout << "[Astar] ERROR: Disconnected Nodes..." << endl;
			return -1;
		}
		cout << "Now: "<< cur_idx << " Goal: " << target_index << endl;
		update_visibility(cur_idx, target_index, size, node_list, &visible_queue);
		cur_idx = min_dist_idx(node_list, &visible_queue);
		node_list[cur_idx].confirmed = true;
	} while (cur_idx != target_index); // if target is reachable, end of astar.

	cout << "Astar done. Do backtrace" << endl;

	NodeMap cur_node = node_list[cur_idx];

	while (cur_node.prev_node_idx != start_idx) {
		cout << "Move " << cur_idx << " obj to " << cur_node.prev_node_idx << " obj." << endl;
		if (cur_node.prev_node_idx == -1){
			cout << "ERROR : No where to move for object " << cur_idx << endl;
			return -1;
		}
		cur_idx = cur_node.prev_node_idx;
		cur_node = node_list[cur_idx];
	}

	cout << "[Astar] Backtrace done. move to " << cur_node.x << ", " << cur_node.y << endl;
	// visualize(size,	node_list, target_index, 0, 0);
	return cur_idx;
	// Compute visibility & iterate
}

void visualize(int size, NodeMap* nodes, int goal_index, int targetX, int targetY){
	cout << "Visualization" << endl;
    missionmap = cv::Mat::zeros(MAP_HEIGHT, MAP_WIDTH, CV_8UC3);
    int x,y, robot_index;
    // int GAP = 50;

    circle(missionmap, Point(targetX, targetY), 10, Scalar(255,255,255), 2, -1, 0);

    for(int i=0; i<size; i++){
    	x = nodes[i].x; y = nodes[i].y;
    	Scalar color;
    	switch (nodes[i].type){
    		case ROBOT:
    			color = Scalar(255,255,0);
    			robot_index = i;
    			break;
    		case BALL:
    			color = Scalar(50,50,255);
    			break;
    		case PILLAR:
    			color = Scalar(255,255,255);
    			break;
    		case GOAL:
    			color = Scalar(0,255,0);
    	}
    	circle(missionmap, Point(x,y), 5, color, 2, 8, 0);
    }
    for (int i=0; i<pillarCount; i++){
    	circle(missionmap, Point(pillarX[i], pillarY[i]), 18, Scalar(255,255,255), 2, -1, 0);    	
    }


   	int cur_idx = goal_index;
   	if (cur_idx == -1){
	    imshow("BALL HARVESTING MAP", missionmap);
		waitKey(10);   		
   		return;
   	}
   	NodeMap cur_node = nodes[goal_index];
   	cout << "GOAL INDEX " << goal_index << endl;
   	cout << "PREV INDEX " << cur_node.prev_node_idx << endl;
	while (cur_idx != robot_index && cur_node.prev_node_idx != -1) {
		// cout << "Index :" << cur_idx << endl;
		NodeMap prev = nodes[cur_node.prev_node_idx];
		line(missionmap, Point(cur_node.x, cur_node.y), Point(prev.x, prev.y), Scalar(255,255,255), 1, 8, 0);
		cur_idx = cur_node.prev_node_idx;
		cur_node = nodes[cur_idx];
	}
	cout << "[Visualize] MAP CONFIGURATION DONE" << endl;
    imshow("BALL HARVESTING MAP", missionmap);
	waitKey(10);

}
