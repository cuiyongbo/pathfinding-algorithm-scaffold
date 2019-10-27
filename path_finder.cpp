#include <queue>
#include <vector>
#include <climits>
#include <cstddef>
#include "path_finder.h"

using namespace std;

int g_exploredNodes;

GridCoordinate GridCoordinate_make(int x, int y)
{
    GridCoordinate coor;
    coor.x = x;
    coor.y = y;
    return coor;
}

static int nodeId(GridCoordinate coor, int mapWidth)
{
    return coor.x + coor.y*mapWidth;
}

int BFSFindPath(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize)
{
    int n = mapWidth * mapHeight;
    int startNodeId = nodeId(src, mapWidth);
    int targetNodeId = nodeId(dest, mapWidth);

    g_exploredNodes = 0;
    vector<int> parent(n);
    vector<int> distanceFromStart(n, INT_MAX);
    distanceFromStart[startNodeId] = 0;
    queue<int> q;
    q.push(startNodeId);
    vector<int> g_directions {+1, -1, mapWidth, -mapWidth};
    while(!q.empty())
    {
        int u = q.front(); q.pop();
        g_exploredNodes++;

        for(auto e: g_directions)
        {
            int v = u + e;
            if((e==1 && (v%mapWidth==0)) || (e==-1 && (u%mapWidth==0)))
                continue;

            if(0 <= v && v < n && distanceFromStart[v] == INT_MAX && pMap[v])
            {
                parent[v] = u;
                distanceFromStart[v] = distanceFromStart[u] + 1;

                if(v == targetNodeId)
                    goto end;

                q.push(v);
            }
        }
    }

end:
    if(distanceFromStart[targetNodeId] == INT_MAX)
    {
        return -1;
    }
    else if(distanceFromStart[targetNodeId] <= outBufferSize && outBuffer != NULL)
    {
        for(int cur=targetNodeId, i=distanceFromStart[targetNodeId]-1;
            i>=0; --i)
        {
            outBuffer[i] = cur;
            cur = parent[cur];
        }
        return distanceFromStart[targetNodeId];
    }
    else
    {
        return distanceFromStart[targetNodeId];
    }
}

int AStarFindPath(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize)
{

    return 0;
}
