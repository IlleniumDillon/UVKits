#pragma once

#include <iostream>
#include <eigen3/Eigen/Eigen>

namespace UV
{

using namespace Eigen;

const double inf_d = std::numeric_limits<double>::max();
class Status
{
public:
    Status()
    {
        position[0] = 0;
        position[1] = 0;
        position[2] = 0;

        attitude[0] = 0;
        attitude[1] = 0;
        attitude[2] = 0;
        attitude[3] = 0;
    };

    Status(Vector3d& pos)
    {
        position = pos;

        attitude[0] = 0;
        attitude[1] = 0;
        attitude[2] = 0;
        attitude[3] = 0;
    };
    Status(Vector3d& pos, Vector4d& att)
    {
        position = pos;
        attitude = att;
    };

    Status(Vector2d& pos)
    {
        position[0] = pos[0];
        position[1] = pos[1];
        position[2] = 0;
        
        attitude[0] = 0;
        attitude[1] = 0;
        attitude[2] = 0;
        attitude[3] = 0;
    };
    Status(Vector2d& pos, double& att)
    {
        position[0] = pos[0];
        position[1] = pos[1];
        position[2] = 0;

        attitude[0] = att;
        attitude[1] = 0;
        attitude[2] = 0;
        attitude[3] = 0;
    };

public:
    Vector3d& pos(){return position;};
    double& x(){return position.x();};
    double& y(){return position.y();};
    double& z(){return position.z();};

    Vector4d& att(){return attitude;};
    double& w(){return attitude.x();};

private:
    Vector3d position;
    Vector4d attitude;
};
} // namespace UV

