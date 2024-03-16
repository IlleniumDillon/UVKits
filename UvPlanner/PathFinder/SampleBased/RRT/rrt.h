#pragma once

// #include <Eigen/Eigen>
// #include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include "GridNode.hpp"
#include "raycast.h"
#include <random>
namespace UV
{


class Node_Info
{
public:
    Node_Info(double x_, double y_,double z_):x(x),y(y),z(z){}
    double x = 0;
    double y = 0;
    double z = 0;
};

class RRT_Node 
{
public:
    RRT_Node* fatherPtr = nullptr;
    Node_Info data;

    RRT_Node(Node_Info data_, RRT_Node* father_ptr = nullptr):fatherPtr(father_ptr),data(data_){}
//是否将Info变为private，使用eigen
};

class RRT
{
public:
    Node_Info start_data;
    Node_Info goal_data;
    std::vector<RRT_Node*> node_list;
    double step_size;
    
    //下右上xyz
    int mapX=10,mapY=10,mapZ=1;
    double scaleX=1,scaleY=1,scaleZ=1;
    //xyz顺序
    uint8_t* mapData=nullptr;


    int goal_sample_rate;

    std::mt19937 goal_gen;
    std::uniform_int_distribution<int> goal_dis;
    std::uniform_real_distribution<double> x_dis;
    std::uniform_real_distribution<double> y_dis;
    std::uniform_real_distribution<double> z_dis;




    RRT(Node_Info start_data_, Node_Info goal_data_,  //起始、终止点
      double step_size_ = 1.0, int goal_sample_rate_ = 5)  // 障碍物，步长（节点之间的距离），目标采样率（有多少概率直接采样到目标点）
      : start_data(start_data_),
        goal_data(goal_data_),
        step_size(step_size_),
        goal_sample_rate(goal_sample_rate_),
        goal_gen(std::random_device{}()),
        goal_dis(std::uniform_int_distribution<int>(0, 100)),
        x_dis(std::uniform_real_distribution<double>(0, mapX*scaleX)),
        y_dis(std::uniform_real_distribution<double>(0, mapY*scaleY)),
        z_dis(std::uniform_real_distribution<double>(0, mapZ*scaleZ))      
        {
            RRT_Node* start_node = new RRT_Node(start_data);
            node_list.push_back(start_node);
        }
    RRT_Node* Sample();
    RRT_Node* Near(RRT_Node* x_rand);
    RRT_Node* Step(RRT_Node* x_rand, RRT_Node* x_near);
    bool CollisionFree(RRT_Node* x_new, RRT_Node* x_near);
    void AddNode(RRT_Node* x_new, RRT_Node* x_father);
    bool SuccessCheck(RRT_Node* x_new);

    std::vector<RRT_Node*> Planning();

};

}

