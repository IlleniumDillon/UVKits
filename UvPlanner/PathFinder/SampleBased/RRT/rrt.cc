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
    double new_scale_x = x_new->data.x / scaleX;
    double new_scale_y = x_new->data.y / scaleY;
    double new_scale_z = x_new->data.z / scaleZ;
    double near_scale_x = x_new->data.x / scaleX;
    double near_scale_y = x_new->data.y / scaleY;
    double near_scale_z = x_new->data.z / scaleZ;

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
    int stepX = (int)signum((int)dx);
    int stepY = (int)signum((int)dy);
    int stepZ = (int)signum((int)dz);

    // See description above. The initial values depend on the fractional
    // part of the origin.
    double tMaxX = intbound(new_scale_x, dx);
    double tMaxY = intbound(new_scale_y, dy);
    double tMaxZ = intbound(new_scale_z, dz);

    // The change in t when taking a step (always positive).
    double tDeltaX = ((double)stepX) / dx;
    double tDeltaY = ((double)stepY) / dy;
    double tDeltaZ = ((double)stepZ) / dz;

    // Avoids an infinite loop.
    if (stepX == 0 && stepY == 0 && stepZ == 0)
        return;

    double dist = 0;
    while (true)
    {
        if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z())
        {
            output[output_points_cnt](0) = x;
            output[output_points_cnt](1) = y;
            output[output_points_cnt](2) = z;

            output_points_cnt++;
            dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                        (z - start(2)) * (z - start(2)));

            if (dist > maxDist)
                return;

            /*            if (output_points_cnt > 1500) {
                            std::cerr << "Error, too many racyast voxels." <<
               std::endl;
                            throw std::out_of_range("Too many raycast voxels");
                        }*/
        }

        if (x == endX && y == endY && z == endZ)
            break;

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

}