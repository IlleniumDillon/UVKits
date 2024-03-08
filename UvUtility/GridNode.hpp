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
    GridNode(Status st)
    {
        status = st;
        comeFrom = nullptr;
    };
public: 
    Status status;
    Vector3i index;
    GridNodePtr comeFrom;
    bool isObs;
};
} // namespace UV
