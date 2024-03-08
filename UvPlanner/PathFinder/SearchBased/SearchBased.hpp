#pragma once

#include <iostream>
#include <vector>

#include "GridNode.hpp"

namespace UV
{

class PathSearcher
{
public:
    virtual void reset() = 0;
    virtual void init(uint8_t* pmap,int l,int w,int h) = 0;
    virtual void 
};

class AStar;

} // namespace UV
