#include "rrt.h"
namespace UV
{

RRT_Node* RRT::Sample()
{
    RRT_Node* x_rand = nullptr;
    if (goal_dis(goal_gen) > goal_sample_rate) { //如果生成的随机数大于目标采样概率，则生成随机点
        double randX = x_dis(goal_gen); //生成随机点X,Y
        double randY = y_dis(goal_gen);
        double randZ = z_dis(goal_gen);
        x_rand = new RRT_Node(Node_Info(randX,randY,randZ));//存储生成的随机位置

    }
    else 
    {   //否则目标点则为随机点
        x_rand = new RRT_Node(goal_data);
    }
    return x_rand;
}

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
RRT_Node* RRT::Step(RRT_Node* x_rand, RRT_Node* x_near)
{
    Eigen::Vector3d direction(x_rand->data.x-x_near->data.x,
                              x_rand->data.y-x_near->data.y,
                              x_rand->data.z-x_near->data.z);
    double dis_norm = direction.squaredNorm();
    Eigen::Vector3d vec_step = step_size/dis_norm*direction;

    RRT_Node* x_new = new RRT_Node(Node_Info(x_near->data.x+vec_step.x(),
                                             x_near->data.y+vec_step.y(),
                                             x_near->data.z+vec_step.z())); 
    return x_new;
}

void RRT::AddNode(RRT_Node* x_new, RRT_Node* x_father)
{
    x_new->fatherPtr = x_father;
    node_list.push_back(x_new);
}

bool RRT::SuccessCheck(RRT_Node* x_new)
{
    Eigen::Vector3d dis(x_new->data.x-goal_data.x,
                        x_new->data.y-goal_data.y,
                        x_new->data.z-goal_data.z);
    if(dis.squaredNorm()<=step_size)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool RRT::CollisionFree(RRT_Node* x_new, RRT_Node* x_near)
{
    double new_scale_x = x_new->data.x / scaleX;
    double new_scale_y = x_new->data.y / scaleY;
    double new_scale_z = x_new->data.z / scaleZ;
    double near_scale_x = x_new->data.x / scaleX;
    double near_scale_y = x_new->data.y / scaleY;
    double near_scale_z = x_new->data.z / scaleZ;

    //得到栅格索引
    int x = (int)std::floor(new_scale_x);
    int y = (int)std::floor(new_scale_y);
    int z = (int)std::floor(new_scale_z);
    int endX = (int)std::floor(near_scale_x);
    int endY = (int)std::floor(near_scale_y);
    int endZ = (int)std::floor(near_scale_z);

    Eigen::Vector3d direction(endX-x,endY-y,endZ-z);
    double maxDist = direction.squaredNorm();

    // Break out direction vector.
    double dx = endX - x;
    double dy = endY - y;
    double dz = endZ - z;

    // Direction to increment x,y,z when stepping.
    int stepX = (int)signum(near_scale_x-new_scale_x);
    int stepY = (int)signum(near_scale_y-new_scale_y);
    int stepZ = (int)signum(near_scale_z-new_scale_z);

    // See description above. The initial values depend on the fractional
    // part of the origin.
    double tMaxX = intbound(new_scale_x, dx);
    double tMaxY = intbound(new_scale_y, dy);
    double tMaxZ = intbound(new_scale_z, dz);

    // The change in t when taking a step (always positive).
    double tDeltaX = ((double)stepX) / dx;
    double tDeltaY = ((double)stepY) / dy;
    double tDeltaZ = ((double)stepZ) / dz;

    double dist = 0;
    while (true)
    {
        //碰到障碍物
        if (mapData[x*mapZ*mapY + y*mapZ + z]==1)
        {
            return false;
        }
        //成功到达
        if (x == endX && y == endY && z == endZ)
        {
            return true;
        }

        // tMaxX stores the t-value at which we cross a cube boundary along the
        // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
        // chooses the closest cube boundary. Only the first case of the four
        // has been commented in detail.
        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                // Update which cube we are now in.
                x += stepX;
                // Adjust tMaxX to the next X-oriented boundary crossing.
                tMaxX += tDeltaX;
            }
            else
            {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                y += stepY;
                tMaxY += tDeltaY;
            }
            else
            {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }
}

std::vector<RRT_Node*> RRT::Planning()
{
    RRT_Node* x_rand  = nullptr;
    RRT_Node* x_near  = nullptr;
    RRT_Node* x_new  = nullptr;
    std::vector<RRT_Node*> output;
    if(SuccessCheck(node_list[0]))
    {
        output.push_back(node_list[0]);
        return output;
    }
    while(true)
    {
        RRT_Node* x_rand = Sample();
        RRT_Node* x_near = Near(x_rand);
        RRT_Node* x_new = Step(x_rand,x_near);
        if(CollisionFree(x_new,x_near))
        {
            AddNode(x_new,x_near);
            delete x_rand;
            delete x_near;
            if(SuccessCheck(x_new))
            {
                break;
            }
        }
        else
        {

            delete x_rand;
            delete x_near;
            delete x_new;

        }

    }
    RRT_Node* x_i = x_new;
    while(x_i->fatherPtr!=nullptr)
    {
        output.push_back(x_i);
        x_i = x_i->fatherPtr;
    }
    output.push_back(x_i);//起点
    return output;
}

}