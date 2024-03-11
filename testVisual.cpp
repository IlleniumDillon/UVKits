#include <iostream>
#include "SearchBased/SearchBased.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace Eigen;
int main()
{
    uint8_t map[] = {
        0,0,0,0,0,0,
        0,1,1,0,0,0,
        0,1,0,0,0,0,
        0,0,0,0,0,0,
    };
    UV::AStar solver;
    solver.setMap(map,4,6);
    solver.setScale(1,1);
    solver.reset();
    Vector2d start(0,0);
    Vector2d goal(3,5);
    solver.solve(start,goal);

    
    std::vector<double> x, y;

    for(int i = solver.path.size()-1; i >=0; i--)
    {
        x.push_back(solver.path.at(i).x());
        y.push_back(solver.path.at(i).y());
    }
    plt::xlim(0,6);
    plt::ylim(0,4);   
    plt::plot(y,x);
    plt::grid(true);

    x.clear();
    y.clear();

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            if(map[i*6+j])
            {
                x.push_back(i);
                y.push_back(j);
                
            }
        }
    }
    plt::plot(x,y,"o");
    plt::show();
    return 0;
}