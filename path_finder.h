#ifndef _PATH_FINDER_H
#define _PATH_FINDER_H

typedef unsigned char uint8_t;

extern int g_exploredNodes;

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

int AStarFindPath(GridCoordinate src, GridCoordinate dest,
    const uint8_t* pMap, int mapWidth, int mapHeight,
    int* outBuffer, int outBufferSize);


#endif
