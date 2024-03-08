#include "SearchBased.hpp"

using namespace UV;

class AStar :   PathSearcher
{
    double gScore = inf_d;
    double fScore = inf_d;
    int flag;
};