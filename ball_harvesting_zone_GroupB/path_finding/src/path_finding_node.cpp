#include <ros/ros.h>

#include "geometry_msgs/PoseArray.h"
//#include "Astar.h"
#include "core_msgs/to_astar.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <ros/package.h>
#include <boost/thread.hpp>

#include <vector>

float x_pos, y_pos, Y_pos;
float goal_x, goal_y, goal_Y;
float Bx[5];
float By[5];

//function definition of Astar


using namespace std;


typedef struct map_t {
	double height; 	double width;
} map_t;

typedef struct car {
	double x0; 	double y0; 	double ang;
} car;

typedef struct pillar {
	double X; 	double Y;
}pillar;

typedef struct ball {
	double X; 	double Y;
}ball;

typedef struct target_t {
	double X;	double Y;	double x;	double y;
}target_t;

typedef struct info {
	map_t MAP;
	car CAR;
	pillar pillar1; pillar pillar2; pillar pillar3; pillar pillar4;
	ball ball1; ball ball2; ball ball3; ball ball4; ball ball5;
	target_t TARGET;
	double gridsize;
	double rad1;
	double rad2;
	bool Extreme1;
	bool Extreme2;
}info;

typedef struct Node_t {
	double x; double y;
}Node_t;

typedef struct PathNode {
	vector<vector<double>> vec = {};
	bool ver;
}PathNode;

typedef struct node_s {
	int G;
	int H;
	int F;
	int x, y;
	struct node_s* direct[8];
	struct node_s* prev_node;
	struct node_s* next_node;

} node_t;

typedef struct node_stack_s {
	node_t* node;
	struct node_stack_s* next_node;
} node_stack_t;


enum {
	DIRECTION_LEFT, DIRECTION_LEFTUP, DIRECTION_UP, DIRECTION_RIGHTUP, DIRECTION_RIGHT,
	DIRECTION_RIGHTDOWN, DIRECTION_DOWN, DIRECTION_LEFTDOWN
};



node_t* OPEN = NULL, * CLOSED = NULL;
node_stack_t* STACK = NULL;

const int FINDPATH_LIMIT = 1000;

Node_t XY2xyCalculator(double X, double Y, double x0, double y0, double ang) {
	Node_t node;
	node.x = cos(ang) * (X - x0) - sin(ang) * (Y - y0);
	node.y = sin(ang) * (X - x0) + cos(ang) * (Y - y0);
	return node;
}

info XY2xyConverter(info infor) {
	Node_t node;
	double x0 = infor.CAR.x0;
	double y0 = infor.CAR.y0;
	double ang = infor.CAR.ang;

	node = XY2xyCalculator(infor.TARGET.X, infor.TARGET.Y, x0, y0, ang);
	infor.TARGET.x = node.x;
	infor.TARGET.y = node.y;
	return infor;
}

void init_astar() {
	node_t* temp;
	node_stack_t* stack;
	while (OPEN) {
		temp = OPEN->next_node;
		free(OPEN);
		OPEN = temp;
	}
	while (CLOSED) {
		temp = CLOSED->next_node;
		free(CLOSED);
		CLOSED = temp;
	}
	while (STACK) {
		stack = STACK->next_node;
		free(STACK);
		STACK = stack;
	}
}

bool distance_available(double x, double y, double x0, double y0, double rad) {
	if (sqrt(pow(x - x0, 2) + pow(y - y0, 2)) > rad) {
		return true;
	};
	return false;
}

bool wall_available(double x, double y, info infor) {
	if (x > 300 && x < infor.MAP.height - 300 && y>300 && y < infor.MAP.width - 300) {
		return true;
	}
	else if ((x > 0 && y >= 300 && x <= 300 && y <= 700) || (x >= infor.MAP.height - 300 && x < infor.MAP.height - 70 && y < 1600 && y>1400)) {
		return true;
	}
	return false;
}

bool is_available_location(double x, double y, info infor) {
	bool tf1 = distance_available(x, y, infor.pillar1.X, infor.pillar1.Y, infor.rad1);
	bool tf2 = distance_available(x, y, infor.pillar2.X, infor.pillar2.Y, infor.rad1);
	bool tf3 = distance_available(x, y, infor.pillar3.X, infor.pillar3.Y, infor.rad1);
	bool tf4 = distance_available(x, y, infor.pillar4.X, infor.pillar4.Y, infor.rad1);
	bool tf5 = distance_available(x, y, infor.ball1.X, infor.ball1.Y, infor.rad2);
	bool tf6 = distance_available(x, y, infor.ball2.X, infor.ball2.Y, infor.rad2);
	bool tf7 = distance_available(x, y, infor.ball3.X, infor.ball3.Y, infor.rad2);
	bool tf8 = distance_available(x, y, infor.ball4.X, infor.ball4.Y, infor.rad2);
	bool tf9 = distance_available(x, y, infor.ball5.X, infor.ball5.Y, infor.rad2);
	bool tf10 = wall_available(x, y, infor);

	if (tf1 && tf2 && tf3 && tf4 && tf5 && tf6 && tf7 && tf8 && tf9 && tf10) {
		return true;
	}
	return false;
}

bool is_available_grid(int x, int y, info infor, int map[101][101]) {
	double xy_x = ((double)x - 50) * infor.gridsize;
	double xy_y = ((double)y - 50) * infor.gridsize;
	double XY_X = +cos(infor.CAR.ang) * xy_x + sin(infor.CAR.ang) * xy_y + infor.CAR.x0;
	double XY_Y = -sin(infor.CAR.ang) * xy_x + cos(infor.CAR.ang) * xy_y + infor.CAR.y0;
	bool tf0 = is_available_location(XY_X, XY_Y, infor);
	bool tf11;
	if (map[x][y] == 0) {
		tf11 = false;
	}
	else {
		tf11 = true;
	}
	if (tf0 && tf11) {
		return true;
	}
	return false;
}

node_t* is_open(int x, int y) {

	node_t* temp = OPEN;

	while (temp) {
		if (temp->x == x && temp->y == y)
			return temp;
		temp = temp->next_node;
	}

	return NULL;
}

node_t* is_closed(int x, int y) {
	node_t* temp = CLOSED;
	while (temp) {
		if (temp->x == x && temp->y == y)
			return temp;
		temp = temp->next_node;
	}
	return NULL;
}

void push_into_stack(node_t* node) {
	node_stack_t* temp;
	temp = (node_stack_t*)calloc(1, sizeof(node_stack_t));
	temp->node = node;
	temp->next_node = STACK;
	STACK = temp;
}

node_t* pop_from_stack() {
	node_t* temp;
	node_stack_t* stack;
	stack = STACK;
	temp = stack->node;
	STACK = stack->next_node;
	free(stack);
	return temp;
}

void make_sort(node_t* old) {
	node_t* direct, * previousNode;
	int i;
	int G = old->G + 1;
	for (i = 0; i < 8; i++) {
		if ((direct = old->direct[i]) == NULL)
			continue;
		if (direct->G > G) {
			direct->prev_node = old;
			direct->G = G;
			direct->F = direct->H + direct->G;
			push_into_stack(direct);
		}
	}
	while (STACK) {
		previousNode = pop_from_stack();
		for (i = 0; i < 8; i++) {
			if ((direct = previousNode->direct[i]) == NULL)
				break;
			if (direct->G > previousNode->G + 1) {
				direct->prev_node = previousNode;
				direct->G = previousNode->G + 1;
				direct->F = direct->H + direct->G;
				push_into_stack(direct);
			}
		}
	}
}

void insert_node(node_t* present) {
	node_t* old = NULL, * temp = NULL;
	if (OPEN == NULL) {
		OPEN = present;
		return;
	}
	temp = OPEN;
	//OPEN에 있는 노드가 현재 자식 노드보다 평가치가 낮으면
	//OPEN노드를 추적해서 현재 자식 노드보다 평가치가 높은 노드를 찾는다.
	while (temp && (temp->F < present->F)) {
		old = temp;
		temp = temp->next_node;
	}
	//낮은 평가치의 OpenNode 중 출발지에 가장 가까운 노드 -> 추가할 자식 노드 -> 높은 평가치의 OpenNode중 출발지에 가장 먼 노드
	if (old) {
		present->next_node = temp;
		old->next_node = present;
	}
	else {
		present->next_node = temp;
		OPEN = present;
	}

}

void extend_child_node(node_t* node, int x, int y, int dest_x, int dest_y, int cur_direct) {
	node_t* old = NULL, * child = NULL;
	int G = node->G + 1;
	if (old = is_open(x, y)) {
		node->direct[cur_direct] = old;
		//그리고 노드 재연결 및 설정 . 참고로 노드 방향이 반대로 적용된다는 걸 고려할 것.
		if (G < old->G) {
			old->prev_node = node;
			old->G = G;
			old->F = old->H + old->G;
		}//if()

	}
	else if (old = is_closed(x, y)) {
		node->direct[cur_direct] = old;
		//어떤 경우에는 기존 CloseNode에 있는 노드의 G가 더 적을 경우가 발생할 수도 있는데
		//이 때 순서를 다시 설정
		if (G < old->G) {
			old->prev_node = node;
			old->G = G;
			old->F = old->H + old->G;
			make_sort(old);
		}//if()

	//확장할 노드 생성 및 OpenNode 리스트에 추가
	}
	else {
		if ((child = (node_t*)calloc(1, sizeof(node_t))) == NULL)
			return;
		child->prev_node = node;
		child->G = G;
		child->H = (x - dest_x) * (x - dest_x) + (y - dest_y) * (y - dest_y);
		child->F = child->H + child->G;
		child->x = x;
		child->y = y;
		insert_node(child);
		node->direct[cur_direct] = child;
	}
}

char make_child(node_t* node, int dest_x, int dest_y, info infor, int map[101][101]) {
	int x, y;
	char flag = 0;
	char checkis_available_grid[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	x = node->x;
	y = node->y;
	//이동 가능한지 여부 판단.
	checkis_available_grid[DIRECTION_RIGHTDOWN] = is_available_grid(x, y + 1, infor, map);
	checkis_available_grid[DIRECTION_DOWN] = is_available_grid(x, y + 1, infor, map);
	checkis_available_grid[DIRECTION_LEFTDOWN] = is_available_grid(x - 1, y + 1, infor, map);
	checkis_available_grid[DIRECTION_LEFT] = is_available_grid(x - 1, y, infor, map);
	checkis_available_grid[DIRECTION_LEFTUP] = is_available_grid(x - 1, y - 1, infor, map);
	checkis_available_grid[DIRECTION_UP] = is_available_grid(x, y - 1, infor, map);
	checkis_available_grid[DIRECTION_RIGHTUP] = is_available_grid(x + 1, y - 1, infor, map);
	checkis_available_grid[DIRECTION_RIGHT] = is_available_grid(x + 1, y, infor, map);
	//각 이동 가능한 노드에 대한 자식 노드 생성
	if (checkis_available_grid[DIRECTION_LEFT]) {
		extend_child_node(node, x - 1, y, dest_x, dest_y, 0);
		flag = 1;
	}
	if (checkis_available_grid[DIRECTION_RIGHT]) {
		extend_child_node(node, x + 1, y, dest_x, dest_y, 4);
		flag = 1;
	}
	if (checkis_available_grid[DIRECTION_UP]) {
		extend_child_node(node, x, y - 1, dest_x, dest_y, 2);
		flag = 1;
	}
	if (checkis_available_grid[DIRECTION_DOWN]) {
		extend_child_node(node, x, y + 1, dest_x, dest_y, 6);
		flag = 1;
	}
	//if (checkis_available_grid[DIRECTION_RIGHTDOWN] && checkis_available_grid[DIRECTION_RIGHT] && checkis_available_grid[DIRECTION_DOWN]) {
	//	extend_child_node(node, x + 1, y + 1, dest_x, dest_y, 5);
	//	flag = 1;
	//}

	//if (checkis_available_grid[DIRECTION_LEFTUP] && checkis_available_grid[DIRECTION_LEFT] && checkis_available_grid[DIRECTION_UP]) {
	//	extend_child_node(node, x - 1, y - 1, dest_x, dest_y, 1);
	//	flag = 1;
	//}

	//if (checkis_available_grid[DIRECTION_RIGHTUP] && checkis_available_grid[DIRECTION_RIGHT] && checkis_available_grid[DIRECTION_UP]) {
	//	extend_child_node(node, x + 1, y - 1, dest_x, dest_y, 3);
	//	flag = 1;
	//}

	//if (checkis_available_grid[DIRECTION_LEFTDOWN] && checkis_available_grid[DIRECTION_LEFT] && checkis_available_grid[DIRECTION_DOWN]) {
	//	extend_child_node(node, x - 1, y + 1, dest_x, dest_y, 7);
	//	flag = 1;
	//}
	if (checkis_available_grid[DIRECTION_RIGHTDOWN]) {
		extend_child_node(node, x + 1, y + 1, dest_x, dest_y, 5);
		flag = 1;
	}
	if (checkis_available_grid[DIRECTION_LEFTUP]) {
		extend_child_node(node, x - 1, y - 1, dest_x, dest_y, 1);
		flag = 1;
	}
	if (checkis_available_grid[DIRECTION_RIGHTUP]) {
		extend_child_node(node, x + 1, y - 1, dest_x, dest_y, 3);
		flag = 1;
	}
	if (checkis_available_grid[DIRECTION_LEFTDOWN]) {
		extend_child_node(node, x - 1, y + 1, dest_x, dest_y, 7);
		flag = 1;
	}
	return flag;
}//char MakeChild(node_t* node, int dest_x, int dest_y)

node_t* find_path(int start_x, int start_y, int dest_x, int dest_y, info infor, int map[101][101]) {
	node_t* present, * best = NULL;
	int count = 0;
	//시작 위치 노드 추가
	present = (node_t*)calloc(1, sizeof(node_t));
	present->G = 0;
	present->H = (int)sqrt(pow(dest_x - start_x, 2) + pow(dest_y - start_y, 2));
	present->F = present->H;
	present->x = dest_x;
	present->y = dest_y;
	OPEN = present;
	while (count < FINDPATH_LIMIT) {
		if (OPEN == NULL)
			return best;
		best = OPEN;					    //매 루프문을 돌면서 OpenNode의 후보로부터 탐색을 우선 시도합니다.
		OPEN = best->next_node;             //그리고 그 다음 노드가 OpenNode의 최상위 후보노드로 지정되며
		best->next_node = CLOSED;           //지금까지 구축된 최적의 노드를 이번에 탐색할 best노드와 연결함으로써
                                            //현재까지 탐색된  최적의 길을 유지하게 됩니다.
		CLOSED = best;						//이 best노드는 이번 루프에서 탐색 시도되므로 close노드로 들어가게 되는거죠.
		if (best == NULL)
			return NULL;
		//목적지 도착.
		if (best->x == start_x && best->y == start_y)
			return best;
		if (make_child(best, start_x, start_y, infor, map) == 0 && count == 0)
			return NULL;
		count++;
	}
	return best;
}

bool NodeVerification(vector<vector<double>> vec, info infor) {
	int N = vec[0].size();
	double x = vec[0][0]*1000 -3000;
	double y = vec[1][0]*1000;

	if (sqrt(pow(x - infor.CAR.x0, 2) + pow(y - infor.CAR.y0, 2)) < infor.gridsize) {
		return true;
	}
	else {
		return false;
	}
}

info get_infor_target(info infor) {

  infor.TARGET.X=goal_x*1000-3000;
  infor.TARGET.Y=goal_y*1000;

  infor = XY2xyConverter(infor);
  return infor;
}

PathNode Astar(info infor, double radius1, double radius2) {
	infor.rad1 = radius1;
	infor.rad2 = radius2;
	infor.gridsize = 100; // change gridsize

	Node_t node;
	Node_t xycoor[101][101];
	for (int i = 0; i < 101; ++i) {
		for (int j = 0; j < 101; ++j) {
			node.x = (i - 50) * infor.gridsize;
			node.y = (j - 50) * infor.gridsize;
			xycoor[i][j] = node;
		}
	}
	Node_t XYcoor[101][101];
	for (int i = 0; i < 101; ++i) {
		for (int j = 0; j < 101; ++j) {
			node.x = infor.CAR.x0 + cos(infor.CAR.ang) * xycoor[i][j].x + sin(infor.CAR.ang) * xycoor[i][j].y;
			node.y = infor.CAR.y0 - sin(infor.CAR.ang) * xycoor[i][j].x + cos(infor.CAR.ang) * xycoor[i][j].y;
			XYcoor[i][j] = node;
		}
	}

	int map[101][101] = {};
	for (int i = 0; i < 101; ++i) {
		for (int j = 0; j < 101; ++j) {
			if (i == 0 || i == 100 || j == 0 || j == 100) {
				map[i][j] = 0;
			}
			else {
				map[i][j] = 1;
			}
		}
	}

	for (int i = 0; i < 101; ++i) {
		for (int j = 0; j < 101; ++j) {
			if (map[i][j] == 1) {
				if (not is_available_location(XYcoor[i][j].x, XYcoor[i][j].y, infor)) {
					map[i][j] = 0;
				}
			}
		}
	}

	infor = get_infor_target(infor);
	int start_x = 50;
	int start_y = 50;
	int dest_x = 50 + infor.TARGET.x / infor.gridsize;
	int dest_y = 50 + infor.TARGET.y / infor.gridsize;

	node_t* best = find_path(start_x, start_y, dest_x, dest_y, infor, map);

	double temp_x, temp_y;
	vector <double> xvector, yvector;
	double xvec, yvec;

	while (best) {
		temp_x = best->x;
		temp_y = best->y;

		xvec = (cos(infor.CAR.ang) * ((double)temp_x - 50) + sin((double)infor.CAR.ang) * (temp_y - 50)) * infor.gridsize + infor.CAR.x0;
		yvec = (-sin(infor.CAR.ang) * ((double)temp_x - 50) + cos((double)infor.CAR.ang) * (temp_y - 50)) * infor.gridsize + infor.CAR.y0;
		best = best->prev_node;
		xvector.push_back((xvec+3000)/1000);
		yvector.push_back(yvec/1000);
	}

	PathNode AstarNode;
	AstarNode.vec.push_back(xvector);
	AstarNode.vec.push_back(yvector);

	AstarNode.ver = NodeVerification(AstarNode.vec, infor);

	return AstarNode;
}

double length(double x1, double y1, double x2, double y2) {
	return (pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

info Extreme1_handle(info infor) {
	double x = infor.CAR.x0, y = infor.CAR.y0;
	if (length(x, y, infor.ball1.X, infor.ball1.Y) < infor.rad2) infor.ball1.X = -500; infor.ball1.Y = -500;
	if (length(x, y, infor.ball2.X, infor.ball2.Y) < infor.rad2) infor.ball2.X = -500; infor.ball2.Y = -500;
	if (length(x, y, infor.ball3.X, infor.ball3.Y) < infor.rad2) infor.ball3.X = -500; infor.ball3.Y = -500;
	if (length(x, y, infor.ball4.X, infor.ball4.Y) < infor.rad2) infor.ball4.X = -500; infor.ball4.Y = -500;
	if (length(x, y, infor.ball5.X, infor.ball5.Y) < infor.rad2) infor.ball5.X = -500; infor.ball5.Y = -500;
	return infor;
}

info Extreme2_handle(info infor) {
	double x = infor.TARGET.X, y = infor.TARGET.Y;
	if (length(x, y, infor.ball1.X, infor.ball1.Y) < infor.rad2) infor.ball1.X = -500; infor.ball1.Y = -500;
	if (length(x, y, infor.ball2.X, infor.ball2.Y) < infor.rad2) infor.ball2.X = -500; infor.ball2.Y = -500;
	if (length(x, y, infor.ball3.X, infor.ball3.Y) < infor.rad2) infor.ball3.X = -500; infor.ball3.Y = -500;
	if (length(x, y, infor.ball4.X, infor.ball4.Y) < infor.rad2) infor.ball4.X = -500; infor.ball4.Y = -500;
	if (length(x, y, infor.ball5.X, infor.ball5.Y) < infor.rad2) infor.ball5.X = -500; infor.ball5.Y = -500;
	return infor;
}

info initiate_infor(float pos_x, float pos_y, float pos_Y, float b1x, float b2x, float b3x, float b4x, float b5x, float b1y, float b2y, float b3y, float b4y, float b5y) {
	info infor;

	//fixed information :
	infor.MAP.height = 5000;
	infor.MAP.width = 3000;
	infor.pillar1.X = 1000;	//pillar locations
	infor.pillar1.Y = 1500;
	infor.pillar2.X = 2500;
	infor.pillar2.Y = 700;
	infor.pillar3.X = 2500;
	infor.pillar3.Y = 2300;
	infor.pillar4.X = 4000;
	infor.pillar4.Y = 1500;
	infor.Extreme1 = false;
	infor.Extreme2 = false;

    // information from ball detection node and icp node
	infor.CAR.x0 = pos_x*1000-3000;           //current car location(x)[mm]
	infor.CAR.y0 = pos_y*1000;	        //Origin of this information is (3.0m, 0.0m)
	infor.CAR.ang = pos_Y;          //Current car facing direction
	infor.ball1.X = b1x*1000-3000;      //Ball Locations(origin is 3.0m, 0.0m)
	infor.ball1.Y = b1y*1000;
	infor.ball2.X = b2x*1000-3000;
	infor.ball2.Y = b2y*1000;
	infor.ball3.X = b3x*1000-3000;
	infor.ball3.Y = b3y*1000;
	infor.ball4.X = b4x*1000-3000;
	infor.ball4.Y = b4y*1000;
	infor.ball5.X = b5x*1000-3000;
	infor.ball5.Y = b5y*1000;

	return infor;
}
int odom=0; int checker=0;

void astar_Callback(const core_msgs::to_astar::ConstPtr& data)
{
   checker=1;
   int number=data->balls_x.size();
   goal_x=data->destination[0];
   goal_y=data->destination[1];
   goal_Y=data->destination[2];

   for(int i=0; i<number; i++){
     Bx[i]=data->balls_x[i];
     By[i]=data->balls_y[i];
  }
  for(int i=number; i<5; i++){
     Bx[i]=-500;
     By[i]=-500;
  }
	cout<<"goal_x"<<goal_x<<" ,goal_y "<<goal_y<<" ,goal_Y "<<goal_Y<<endl;
}

void odom_Callback(const geometry_msgs::Vector3::ConstPtr& odometry)
{
   odom=1;
    x_pos = odometry->x;
    y_pos = odometry->y;
    Y_pos = odometry->z;
    cout<<"x"<<x_pos<<" ,y "<<y_pos<<" ,Y "<<Y_pos<<endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_finding_node");
    ros::NodeHandle nh;

    ros::Subscriber ball_position_sub = nh.subscribe<core_msgs::to_astar>("/to_astar", 1, astar_Callback);
    ros::Subscriber odometry_sub = nh.subscribe<geometry_msgs::Vector3>("/odometry", 1, odom_Callback);

    ros::Publisher checkpoint_pub = nh.advertise<geometry_msgs::PoseArray>("/checkpoint", 1);

    info infor;		//This struct contains all information of car, vehicle, ball locations of models
    PathNode AstarNode;
    geometry_msgs::PoseArray posearray;
    geometry_msgs::Pose p;

    while(ros::ok){
      ros::spinOnce();
      if(odom==1 && checker==1){
         odom=0; checker=0;
         if(goal_x==-1 && goal_y==-1){
	p.position.x=x_pos;
	p.position.y=y_pos;
	p.position.z=Y_pos;
	posearray.poses.push_back(p);

	p.position.x=-1;
	p.position.y=-1;
	if(goal_Y==100) p.position.z=Y_pos;
	else p.position.z=goal_Y;
	posearray.poses.push_back(p);
	checkpoint_pub.publish(posearray);
	posearray.poses.clear();
          }
          else{

	infor = initiate_infor(x_pos, y_pos, Y_pos, Bx[0], Bx[1], Bx[2], Bx[3], Bx[4], By[0], By[1], By[2], By[3], By[4]);

	//Assume possible node path exists(1).
	for (int radius = 550; radius >= 500; radius = radius - 25) {
		init_astar();
		AstarNode = Astar(infor, radius, radius);
		if (AstarNode.ver) {
			break;
		}
	}

	//Assume possible node path exists(2)
	if (!AstarNode.ver) {
		for (int radius = 475; radius >= 425; radius = radius - 25) {
			init_astar();
			AstarNode = Astar(infor, 500, radius);
			if (AstarNode.ver) {
				break;
			}
		}
	}
	//if there exists no path due to wierd red ball location,
	//nelgect red balls
	if (!AstarNode.ver) {
		init_astar();
		AstarNode = Astar(infor, 500, -1);
	}
	//if avobe approaches of finding a* node path is not succesful,
	//devide some possible cases to "Extreme1" and "Extreme2"
	//"Extreme1" : if current car location is on "not allowed" region of map
	//"Extreme2" : if target location is on "not allowed" region of map
	if (!AstarNode.ver) {
		if (!is_available_location(infor.CAR.x0, infor.CAR.y0, infor)) infor.Extreme1 = true;
		if (!is_available_location(infor.TARGET.X, infor.TARGET.Y, infor)) infor.Extreme2 = true;
	}
	//if such "Extreme"s is due to red ball location, nelgect the red ball.
	if (!AstarNode.ver) {
		infor.rad1 = 500; infor.rad2 = 425;
		if (infor.Extreme1) infor = Extreme1_handle(infor);
		if (infor.Extreme2) infor = Extreme2_handle(infor);

		init_astar();
		AstarNode = Astar(infor, 500, 425);
	}
	//for above case, if path does not exist due to red ball location, neglect red ball again.
	if (!AstarNode.ver) {
		init_astar();
		AstarNode = Astar(infor, 500, -1);
	}
	//if "Extreme"s are due to pillar location, try "tightest" allowance to find the path
	if (!AstarNode.ver) {
		init_astar();
		AstarNode = Astar(infor, 425, 425);
	}
	//Astar path finding ends;

	//if we found Astar node path, print the result.
	if (AstarNode.ver) {
		for (int i = 0; i < AstarNode.vec[0].size() - 1; i++) {
			cout << "(" << AstarNode.vec[0][i] << ", " << AstarNode.vec[1][i] << ")" << ", ";

			p.position.x=AstarNode.vec[0][i];
			p.position.y=AstarNode.vec[1][i];
			if (i==0) p.position.z=Y_pos;
			else p.position.z=0;
			posearray.poses.push_back(p);

		}
		p.position.x=goal_x;
		p.position.y=goal_y;
		p.position.z=goal_Y;
		posearray.poses.push_back(p);

		checkpoint_pub.publish(posearray);
		posearray.poses.clear();
	}
  else{
		cout << "no" <<endl;
	}
	//AstarNode is Astar's output, and it consists with
	//AstarNode.vec (vector<vector<double>> type)
		//: AstarNode.vec[0] is X coordinate and AstarNode.vec[1] is Y coordinate.
		//: Since Astar algorithm's origin is (3m, 0m),
		//: This output may be changed to (X+3000, Y)[mm] or ((X+3000)/1000, Y/1000)[m]
        }

     }
    }

    return 0;
}
