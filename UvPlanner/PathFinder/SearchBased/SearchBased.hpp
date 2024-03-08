#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "GridNode.hpp"

using std::min;
using std::max;

namespace UV
{

typedef std::function<double(Status,Status)> HeuFunction;
typedef std::function<void(Status ,std::vector<Status> & ,std::vector<double> & )> ExpandFunction;
class PathSearcher
{
public:
    virtual void reset() = 0;
    virtual void setMap(uint8_t* pmap,int l,int w,int h=1) = 0;
    virtual void setScale(double ls, double ws, double hs=1){scaleLenth=ls,scaleWidth=ws,scaleHight=hs;};
    void setHeu(HeuFunction h){getHeu = h;};
    void setExpand(ExpandFunction e){getNeighbour = e;};
    //virtual void setObs(int x,int y,int z=0);
    //virtual void plan(GridNode& start, GridNode& goal) = 0;
    virtual bool solve(Status& start, Status& goal) = 0;
    Status cvtIndex2Status(Vector3i & index)
    {
        Status st;

        st.x() = ((double)index(0) + 0.5) * scaleLenth;
        st.y() = ((double)index(1) + 0.5) * scaleWidth;
        st.z() = ((double)index(2) + 0.5) * scaleHight;

        return st;
    }
    Vector3i cvtStatus2Index(Status & st)
    {
        Vector3i idx;
        idx <<  int( st.x() * scaleLenth),
                int( st.y() * scaleWidth),
                int( st.z() * scaleHight);                  
  
        return idx;
    }


protected:
    int mapLength=0,mapWidth=0,mapHight=0;
    double scaleLenth=1,scaleWidth=1,scaleHight=1;
    HeuFunction getHeu;
    ExpandFunction getNeighbour;
    uint8_t* mapData=nullptr;
    std::vector<Status> path;
};
} // namespace UV

#include "AStar.hpp"
