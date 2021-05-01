#include <iostream>
#include <vector>

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

float 
enum {
	DIRECTION_LEFT, DIRECTION_LEFTUP, DIRECTION_UP, DIRECTION_RIGHTUP, DIRECTION_RIGHT,
	DIRECTION_RIGHTDOWN, DIRECTION_DOWN, DIRECTION_LEFTDOWN
}



node_t* OPEN = NULL, * CLOSED = NULL;
node_stack_t* STACK = NULL;

const int FINDPATH_LIMIT = 1000;

Node_t XY2xyCalculator(double X, double Y, double x0, double y0, double ang);

info XY2xyConverter(info infor);

void init_astar();

bool distance_available(double x, double y, double x0, double y0, double rad);

bool wall_available(double x, double y, info infor);

bool is_available_location(double x, double y, info infor);

bool is_available_grid(int x, int y, info infor, int map[101][101]);

node_t* is_open(int x, int y);

node_t* is_closed(int x, int y);

void push_into_stack(node_t* node);

node_t* pop_from_stack();

void make_sort(node_t* old);

void insert_node(node_t* present);

void extend_child_node(node_t* node, int x, int y, int dest_x, int dest_y, int cur_direct);

char make_child(node_t* node, int dest_x, int dest_y, info infor, int map[101][101]);

node_t* find_path(int start_x, int start_y, int dest_x, int dest_y, info infor, int map[101][101]);

bool NodeVerification(vector<vector<double>> vec, info infor);

PathNode Astar(info infor, double radius1, double radius2);

double length(double x1, double y1, double x2, double y2);

info Extreme1_handle(info infor);

info Extreme2_handle(info infor);

info get_infor_target(info infor);
