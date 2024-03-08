#pragma once

// #include <Eigen/Eigen>
// #include <Eigen/StdVector>
#include <iostream>
#include <vector>

class Node_Info
{
    Node_Info(double x_, double y_):x(x),y(y){}
    double x = 0;
    double y = 0;
};

class RRT_Node
{
public:
    RRT_Node* fatherPtr = nullptr;
    Node_Info data;

    RRT_Node(Node_Info data_, RRT_Node* father_ptr);
//是否将Info变为private，使用eigen
};

class RRT
{
public:
    RRT_Node* start_node;
    RRT_Node* goal_node;
    std::vector<RRT_Node*> node_list;
    double step_size;

    RRT();
    RRT_Node* Sample();
    RRT_Node* Near(RRT_Node* x_rand);
    RRT_Node* Step(RRT_Node* x_rand, RRT_Node* x_near);
    bool CollisionFree(RRT_Node* x_new, RRT_Node* x_near);
    void AddNode(RRT_Node* x_new);
    bool SuccessCheck(RRT_Node* x_new);

    std::vector<RRT_Node*> Planning();

};