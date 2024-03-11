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
                nodeMap[height][width][length]->cameFrom = nullptr;
                nodeMap[height][width][length]->flag = 0;
                nodeMap[height][width][length]->fScore = inf_d;
                nodeMap[height][width][length]->gScore = inf_d;
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

bool AStar::solve(Status start, Status goal)
{
    Vector3i startIndex = cvtStatus2Index(start);
    Vector3i endIndex   = cvtStatus2Index(goal);

    AStarNodePtr startPtr = new AStarNode(startIndex);
    AStarNodePtr endPtr   = new AStarNode(endIndex);
    AStarNodePtr currentPtr = nullptr;
    std::cout << startPtr->index << std::endl <<  std::endl;
    std::cout << endPtr->index << std::endl <<  std::endl;
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr->index,endPtr->index);
    startPtr -> flag = 1; 

    startPtr->status = start;
    endPtr->status = goal;

    openSet.insert(std::make_pair(startPtr -> fScore, startPtr));

    std::vector<Vector3i> neighborSets;
    std::vector<double> edgeCostSets;

    while ( !openSet.empty() )
    {
        auto lowCostPair = openSet.begin();
        currentPtr = lowCostPair->second;
        openSet.erase(lowCostPair);
        currentPtr->flag = -1;
        std::cout << currentPtr->index << std::endl <<  std::endl;
        if( currentPtr->index == endIndex )
        {
            ///TODO: generate statue flow
            path.push_back(goal);
            if(currentPtr->cameFrom!=nullptr)
                currentPtr = currentPtr->cameFrom;
            while (currentPtr->cameFrom!=nullptr)
            {
                Status p = cvtIndex2Status(currentPtr->index);
                path.push_back(p);
                currentPtr = currentPtr->cameFrom;
            }
            path.push_back(start);
            return true;
        }

        getNeighbour(currentPtr->index,neighborSets,edgeCostSets);

        for(int i = 0; i < (int)neighborSets.size(); i++)
        {
            Vector3i neighborIndex = neighborSets.at(i);
            ///TODO: skip invalid point
            if(neighborIndex.x()<0||neighborIndex.x()>=mapLength||
                neighborIndex.y()<0||neighborIndex.y()>=mapWidth||
                neighborIndex.z()<0||neighborIndex.z()>=mapHight||
                mapData[neighborIndex.z()*mapLength*mapWidth+neighborIndex.x()*mapWidth+neighborIndex.y()])
                continue;
            AStarNodePtr neighborPtr = nodeMap[neighborIndex.z()][neighborIndex.y()][neighborIndex.x()];
            if(neighborPtr->flag == 0)
            {
                neighborPtr->cameFrom = currentPtr;
                
                neighborPtr -> gScore = getHeu(neighborPtr->index,currentPtr->index) + currentPtr->gScore;
                neighborPtr -> fScore = getHeu(neighborPtr->index,endPtr->index)+neighborPtr -> gScore;
        
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
                    neighborPtr->fScore = newGScore + getHeu(neighborPtr->index,endPtr->index);
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

double UV::AStar::getHeu(Vector3i p1, Vector3i p2)
{
    Vector3d delta = Vector3d(p1.x()-p2.x(),p1.y()-p2.y(),p1.z()-p2.z());
    double heu = sqrt(delta.x()*delta.x()+delta.y()*delta.y()+delta.z()*delta.z());
    return heu;
}

void UV::AStar::getNeighbour(Vector3i cur, std::vector<Vector3i> &nlist, std::vector<double> &ncost)
{
    nlist.clear();
    ncost.clear();
    for(int dz = -1; dz <= 1; dz++)
    {
        for(int dy = -1; dy <= 1; dy++)
        {
            for(int dx = -1; dx <= 1; dx++)
            {
                if(dz==0&&dy==0&&dx==0) continue;
                Vector3i n = cur + Vector3i(dx,dy,dz);
                nlist.push_back(n);
                ncost.push_back(sqrt(dx*dx+dy*dy+dz*dz));
            }
        }
    }
}
