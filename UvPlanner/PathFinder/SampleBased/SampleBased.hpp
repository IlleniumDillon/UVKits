#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "GridNode.hpp"

using std::min;
using std::max;

namespace UV
{

// typedef std::function<double(Status,Status)> HeuFunction;
// typedef std::function<void(Status ,std::vector<Status> & ,std::vector<double> & )> ExpandFunction;
// class PathSample
// {
// public:
//     virtual void reset() = 0;
//     virtual void setMap(uint8_t* pmap,int l,int w,int h=1) = 0;
//     virtual void setScale(double ls, double ws, double hs=1){scaleLenth=ls,scaleWidth=ws,scaleHight=hs;};
//     virtual bool solve(Status& start, Status& goal) = 0;

//     int mapLength=0,mapWidth=0,mapHight=0;
//     double scaleLenth=1,scaleWidth=1,scaleHight=1;
//     uint8_t* mapData=nullptr;
//     // std::vector<Status> path;
// };
} // namespace UV

