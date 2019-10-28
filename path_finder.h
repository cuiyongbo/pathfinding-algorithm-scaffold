#ifndef _PATH_FINDER_H
#define _PATH_FINDER_H

#include <vector>

typedef unsigned char uint8_t;

extern int g_exploredNodes;
extern std::vector<int> g_landmarks;
extern std::vector<std::vector<int>> LD;

struct GridCoordinate
{
    int x, y;
};

GridCoordinate GridCoordinate_make(int x, int y);

typedef int (*path_finder_func_t) (GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int BFSFindPath(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int BFSFindPathDiag(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int AStarFindPath(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int AStarFindPathDiag(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int AStarFindPathNoTie(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int AStarFindPathNoTieDiag(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int DijkstraFindPath(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

int AStarFindPathLandmarks(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);

#endif
