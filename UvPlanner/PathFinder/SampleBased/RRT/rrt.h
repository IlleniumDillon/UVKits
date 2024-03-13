#pragma once

// #include <Eigen/Eigen>
// #include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include "GridNode.hpp"
#include "raycast.h"
namespace UV
{


class Node_Info
{
public:
    Node_Info(double x_, double y_):x(x),y(y),z(z){}
    double x = 0;
    double y = 0;
    double z = 0;
};

class RRT_Node 
{
public:
    RRT_Node* fatherPtr = nullptr;
    Node_Info data;

    RRT_Node(Node_Info data_, RRT_Node* father_ptr):fatherPtr(father_ptr),data(data_){}
//是否将Info变为private，使用eigen
};

class RRT
{
public:
    RRT_Node* start_node;
    RRT_Node* goal_node;
    std::vector<RRT_Node*> node_list;
    double step_size;
    
    //下右上xyz
    int mapX=10,mapY=10,mapZ=1;
    double scaleX=1,scaleY=1,scaleZ=0;
    uint8_t* mapData=nullptr;

    RRT();
    RRT_Node* Sample();
    RRT_Node* Near(RRT_Node* x_rand);
    RRT_Node* Step(RRT_Node* x_rand, RRT_Node* x_near);
    bool CollisionFree(RRT_Node* x_new, RRT_Node* x_near);
    void AddNode(RRT_Node* x_new);
    bool SuccessCheck(RRT_Node* x_new);

    std::vector<RRT_Node*> Planning();

};

}

