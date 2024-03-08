#pragma once

#include<iostream>
#include<Status.hpp>
#include<eigen3/Eigen/Eigen>

namespace UV
{

class GridNode;
typedef class GridNode* GridNodePtr;
class GridNode
{
public:
    GridNode(){};
    GridNode(Status st)
    {
        status = st;
    };
public: 
    Status status;
    Vector3i index;
    //GridNodePtr comeFrom;
};
} // namespace UV
