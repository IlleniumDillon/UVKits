#include "rrt.h"
namespace UV
{

RRT_Node* RRT::Near(RRT_Node* x_rand)
{
    int min_index = -1;
    double min_dis2 = std::numeric_limits<double>::max();
    for(int i = 0;i<node_list.size();++i)
    {
        double distance2 = std::pow(node_list[i]->data.x - x_rand->data.x, 2) +
                           std::pow(node_list[i]->data.y - x_rand->data.y, 2) +
                           std::pow(node_list[i]->data.z - x_rand->data.z, 2);
        if(distance2<min_dis2)
        {
            min_dis2 = distance2;
            min_index = i;
        }
    }
    return node_list[min_index];
}

bool RRT::CollisionFree(RRT_Node* x_new, RRT_Node* x_near)
{
    
}

}