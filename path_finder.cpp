#include <queue>
#include <vector>
#include <climits>
#include <cstddef>
#include <functional>

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
    vector<int> directions {+1, -1, mapWidth, -mapWidth};
    while(!q.empty())
    {
        int u = q.front(); q.pop();
        g_exploredNodes++;

        for(auto e: directions)
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

int BFSFindPathDiag(GridCoordinate src, GridCoordinate dest,
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
    vector<int> directions { +1, -1, mapWidth, -mapWidth,
                            -mapWidth-1, -mapWidth+1,
                            mapWidth-1, mapWidth+1 };
    while(!q.empty())
    {
        int u = q.front(); q.pop();
        g_exploredNodes++;

        for(auto e: directions)
        {
            int v = u + e;
            if(((e == 1 || e == -mapWidth+1 || e == mapWidth+1) && (v%mapWidth==0)) ||
               ((e == -1 || e == -mapWidth-1 || e == mapWidth-1) && (u%mapWidth==0)))
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
    auto heuristic = [=](int u)
    {
        int x=u%mapWidth, y=u/mapWidth;
        return abs(x-dest.x) + abs(y-dest.y);
    };

    int n = mapWidth * mapHeight;
    int startNodeId = nodeId(src, mapWidth);
    int targetNodeId = nodeId(dest, mapWidth);

    g_exploredNodes = 0;
    int discoveredNodeCount = 0;
    vector<int> parent(n);
    vector<int> distanceFromStart(n, INT_MAX);
    distanceFromStart[startNodeId] = 0;
    vector<int> directions {-1, 1, -mapWidth, mapWidth};

    // tuple - distanceToDest, discovedNodeCount, curNodeId
    priority_queue<tuple<int, int, int>,
        vector<tuple<int, int, int>>,
        greater<tuple<int, int, int>>> pq;

    pq.push(make_tuple(0+heuristic(startNodeId), 0, startNodeId));
    while(!pq.empty())
    {
        int u = get<2>(pq.top()); pq.pop(); g_exploredNodes++;
        for (int e: directions)
        {
            int v = u+e;
            if((e==1 && (v%mapWidth)==0) || (e==-1 && (u%mapWidth)==0))
                continue;

            if(0<=v && v<n && distanceFromStart[v] > distanceFromStart[u]+1 && pMap[v])
            {
                parent[v] = u;
                distanceFromStart[v] = distanceFromStart[u] + 1;

                if(v == targetNodeId)
                    goto end;

                pq.push(make_tuple(distanceFromStart[v]+heuristic(v), ++discoveredNodeCount, v));
            }
        }
    }

end:
    if(distanceFromStart[targetNodeId] == INT_MAX)
    {
        return -1;
    }
    else if(outBuffer != NULL && distanceFromStart[targetNodeId] <= outBufferSize)
    {
        for (int cur=targetNodeId, i=distanceFromStart[targetNodeId]-1;
                i >= 0; --i)
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

int AStarFindPathNoTie(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize)
{
    auto heuristic = [=](int u)
    {
        int x=u%mapWidth, y=u/mapWidth;
        return abs(x-dest.x) + abs(y-dest.y);
    };

    int n = mapWidth * mapHeight;
    int startNodeId = nodeId(src, mapWidth);
    int targetNodeId = nodeId(dest, mapWidth);

    g_exploredNodes = 0;
    vector<int> parent(n);
    vector<int> distanceFromStart(n, INT_MAX);
    distanceFromStart[startNodeId] = 0;
    vector<int> directions { +1, -1, mapWidth, -mapWidth};

    // tuple - distanceToDest, curNodeId
    priority_queue<pair<int, int>,
        vector<pair<int, int>>,
        greater<pair<int, int>>> pq;

    pq.push(make_pair(0+heuristic(startNodeId), startNodeId));
    while(!pq.empty())
    {
        int u = pq.top().second; pq.pop(); g_exploredNodes++;
        for (int e: directions)
        {
            int v = u+e;
            if(((e == 1) && (v%mapWidth==0)) || ((e == -1) && (u%mapWidth==0)))
                continue;

            if(0<=v && v<n && distanceFromStart[v] > distanceFromStart[u]+1 && pMap[v])
            {
                parent[v] = u;
                distanceFromStart[v] = distanceFromStart[u] + 1;

                if(v == targetNodeId)
                    goto end;

                pq.push(make_pair(distanceFromStart[v]+heuristic(v), v));
            }
        }
    }

end:
    if(distanceFromStart[targetNodeId] == INT_MAX)
    {
        return -1;
    }
    else if(outBuffer != NULL && distanceFromStart[targetNodeId] <= outBufferSize)
    {
        for (int cur=targetNodeId, i=distanceFromStart[targetNodeId]-1;
                i >= 0; --i)
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

int AStarFindPathDiag(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize)
{
    auto heuristic = [=](int u)
    {
        int x=u%mapWidth, y=u/mapWidth;
        return abs(x-dest.x) + abs(y-dest.y);
    };

    int n = mapWidth * mapHeight;
    int startNodeId = nodeId(src, mapWidth);
    int targetNodeId = nodeId(dest, mapWidth);

    g_exploredNodes = 0;
    int discoveredNodeCount = 0;
    vector<int> parent(n);
    vector<int> distanceFromStart(n, INT_MAX);
    distanceFromStart[startNodeId] = 0;

    vector<int> directions { +1, -1,
                            mapWidth, -mapWidth,
                            -mapWidth-1, -mapWidth+1,
                            mapWidth-1, mapWidth+1 };

    // tuple - distanceToDest, discovedNodeCount, curNodeId
    priority_queue<tuple<int, int, int>,
        vector<tuple<int, int, int>>,
        greater<tuple<int, int, int>>> pq;

    pq.push(make_tuple(0+heuristic(startNodeId), 0, startNodeId));
    while(!pq.empty())
    {
        int u = get<2>(pq.top()); pq.pop(); g_exploredNodes++;
        for (int e: directions)
        {
            int v = u+e;
            if(((e == 1 || e == -mapWidth+1 || e == mapWidth+1) && (v%mapWidth==0)) ||
               ((e == -1 || e == -mapWidth-1 || e == mapWidth-1) && (u%mapWidth==0)))
                continue;

            if(0<=v && v<n && distanceFromStart[v] > distanceFromStart[u]+1 && pMap[v])
            {
                parent[v] = u;
                distanceFromStart[v] = distanceFromStart[u] + 1;

                if(v == targetNodeId)
                    goto end;

                pq.push(make_tuple(distanceFromStart[v]+heuristic(v), ++discoveredNodeCount, v));
            }
        }
    }

end:
    if(distanceFromStart[targetNodeId] == INT_MAX)
    {
        return -1;
    }
    else if(outBuffer != NULL && distanceFromStart[targetNodeId] <= outBufferSize)
    {
        for (int cur=targetNodeId, i=distanceFromStart[targetNodeId]-1;
                i >= 0; --i)
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

int AStarFindPathNoTieDiag(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize)
{
    auto heuristic = [=](int u)
    {
        int x=u%mapWidth, y=u/mapWidth;
        return abs(x-dest.x) + abs(y-dest.y);
    };

    int n = mapWidth * mapHeight;
    int startNodeId = nodeId(src, mapWidth);
    int targetNodeId = nodeId(dest, mapWidth);

    g_exploredNodes = 0;
    vector<int> parent(n);
    vector<int> distanceFromStart(n, INT_MAX);
    distanceFromStart[startNodeId] = 0;

    vector<int> directions { +1, -1,
                            mapWidth, -mapWidth,
                            -mapWidth-1, -mapWidth+1,
                            mapWidth-1, mapWidth+1 };

    // tuple - distanceToDest, curNodeId
    priority_queue<pair<int, int>,
        vector<pair<int, int>>,
        greater<pair<int, int>>> pq;

    pq.push(make_pair(0+heuristic(startNodeId), startNodeId));
    while(!pq.empty())
    {
        int u = pq.top().second; pq.pop(); g_exploredNodes++;
        for (int e: directions)
        {
            int v = u+e;
            if(((e == 1 || e == -mapWidth+1 || e == mapWidth+1) && (v%mapWidth==0)) ||
               ((e == -1 || e == -mapWidth-1 || e == mapWidth-1) && (u%mapWidth==0)))
                continue;

            if(0<=v && v<n && distanceFromStart[v] > distanceFromStart[u]+1 && pMap[v])
            {
                parent[v] = u;
                distanceFromStart[v] = distanceFromStart[u] + 1;

                if(v == targetNodeId)
                    goto end;

                pq.push(make_pair(distanceFromStart[v]+heuristic(v), v));
            }
        }
    }

end:
    if(distanceFromStart[targetNodeId] == INT_MAX)
    {
        return -1;
    }
    else if(outBuffer != NULL && distanceFromStart[targetNodeId] <= outBufferSize)
    {
        for (int cur=targetNodeId, i=distanceFromStart[targetNodeId]-1;
                i >= 0; --i)
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
