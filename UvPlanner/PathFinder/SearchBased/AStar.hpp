#pragma once

#include "SearchBased.hpp"

namespace UV
{
class AStarNode;
typedef AStarNode* AStarNodePtr;
class AStarNode
{
public:
    AStarNode(Status st)
    {
        status = st;
        cameFrom = nullptr;
    };
    AStarNode(Vector3i idx)
    {
        index = idx;
        cameFrom = nullptr;
    };
    AStarNode(int x, int y, int z=0)
    {
        index = Vector3i(x,y,z);
        cameFrom = nullptr;
    };
    AStarNodePtr cameFrom;
    int flag=0;
    double gScore=inf_d;
    double fScore=inf_d;
    Status status;
    Vector3i index;
};

typedef AStarNode* AStarNodePtr;

class AStar :   public PathSearcher
{
public:
    void reset();
    void setMap(uint8_t* pmap,int l,int w,int h=1);
    bool solve(Status start, Status goal);
    double getHeu(Vector3i p1, Vector3i p2);
    void getNeighbour(Vector3i cur,std::vector<Vector3i> &nlist,std::vector<double> &ncost);
private:
    std::multimap<double, AStarNodePtr> openSet;
    AStarNodePtr*** nodeMap=nullptr;
};
}