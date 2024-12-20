
#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <tuple>
#include <algorithm>
#include <list>
#include <limits>
#include <set>
#include <tuple>
#include <map>

int INF = std::numeric_limits<int>::max();

#include "/home/ruben/ws_workspace/ws_planning_shelfino/Shelfino_Studio/shelfino_planning/include/shelfino_planning/matplotlibcpp.h"

namespace plt = matplotlibcpp;

// obstacle

std::vector<int> obsX1{3, 7, 7, 3, 3};
std::vector<int> obsY1{3, 3, 10, 10, 3};

std::vector<int> obsX2{11, 15, 15, 11, 11};
std::vector<int> obsY2{8, 8, 17, 17, 8};

//parameters

int NODES = 1400; // 50;
float RADIUS = 3.5;
float startX = 1.0; // 5.0;
float startY = 1.0; // 12.5;
int startId = 0;
float goalX = 18.0; // 15.0;
float goalY = 14.0; // 5.0;
int goalId = NODES;



struct Node
{
    float x;
    float y;
    int id;
};

