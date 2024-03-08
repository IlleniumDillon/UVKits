#include "AStar.hpp"

using namespace UV;

void AStar::reset()
{
    path.clear();
    openSet.clear();
    for(int height = 0; height < mapHight; height++)
    {
        for(int length = 0; length < mapLength; length++)
        {
            for(int width = 0; width < mapWidth; width++)
            {
                nodeMap[height][length][width]->cameFrom = nullptr;
                nodeMap[height][length][width]->flag = 0;
                nodeMap[height][length][width]->fScore = inf_d;
                nodeMap[height][length][width]->gScore = inf_d;
            }
        }
    }
}

void AStar::setMap(uint8_t *pmap, int l, int w, int h)
{
    mapHight = h;
    mapLength = l;
    mapWidth = w;
    if(mapData != nullptr) delete[] mapData;
    mapData = new uint8_t[l*w*h];
    memcpy(mapData,pmap,l*w*h);
    if(nodeMap != nullptr) delete[] nodeMap;
    nodeMap = new AStarNodePtr** [mapHight];
    for(int height = 0; height < mapHight; height++)
    {
        nodeMap[height] = new AStarNodePtr* [mapWidth];
        for(int width = 0; width < mapWidth; width++)
        {
            nodeMap[height][width] = new AStarNodePtr [mapLength];
            for(int length = 0; length < mapLength; length++)
            {
                nodeMap[height][width][length] = new AStarNode(length,width,height);
            }
        }
    }
}

bool AStar::solve(Status &start, Status &goal)
{
    Vector3i startIndex = cvtStatus2Index(start);
    Vector3i endIndex   = cvtStatus2Index(goal);

    AStarNodePtr startPtr = new AStarNode(startIndex);
    AStarNodePtr endPtr   = new AStarNode(endIndex);
    AStarNodePtr currentPtr = nullptr;

    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr->status,endPtr->status);
    startPtr -> flag = 1; 

    std::vector<Status> neighborSets;
    std::vector<double> edgeCostSets;

    while ( !openSet.empty() )
    {
        auto lowCostPair = openSet.begin();
        currentPtr = lowCostPair->second;
        openSet.erase(lowCostPair);
        currentPtr->flag = -1;

        if( currentPtr->index == endIndex )
        {
            ///TODO: generate statue flow
            return true;
        }

        getNeighbour(currentPtr->status,neighborSets,edgeCostSets);

        for(int i = 0; i < (int)neighborSets.size(); i++)
        {
            Vector3i neighborIndex = cvtStatus2Index(neighborSets.at(i));
            ///TODO: skip invalid point
            AStarNodePtr neighborPtr = nodeMap[neighborIndex.z()][neighborIndex.y()][neighborIndex.x()];
            if(neighborPtr->flag == 0)
            {
                neighborPtr->cameFrom = currentPtr;
                
                neighborPtr -> gScore = getHeu(neighborPtr->status,currentPtr->status) + currentPtr->gScore;
                neighborPtr -> fScore = getHeu(neighborPtr->status,endPtr->status)+neighborPtr -> gScore;
        
                neighborPtr -> flag = 1;
                openSet.insert( std::make_pair(neighborPtr -> fScore, neighborPtr));
                continue;
            }
            else if(neighborPtr -> flag == 1)
            {
                double newGScore = currentPtr->gScore + edgeCostSets.at(i);
                if(neighborPtr->gScore > newGScore)
                {
                    auto neighborRange = openSet.equal_range(neighborPtr->fScore);
                    neighborPtr->gScore = newGScore;
                    neighborPtr->fScore = newGScore + getHeu(neighborPtr->status,endPtr->status);
                    neighborPtr->cameFrom = currentPtr;
                    //if(neighborRange.first == end(openSet)) continue;
                    for(auto i = neighborRange.first; i != neighborRange.second; i++)
                    {
                        if(i->second == neighborPtr)
                        {
                            openSet.erase(i);
                            openSet.insert(std::make_pair(neighborPtr->fScore,neighborPtr));
                            break;
                        }
                    }
                }
                continue;
            }
        }
    }

    return false;
}
