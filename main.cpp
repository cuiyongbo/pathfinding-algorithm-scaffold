#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <stdint.h>
#include <random>

#include "path_finder.h"

using namespace std;

int main(int argc, char const *argv[])
{
    string temp;
    int width, height;
    getline(cin, temp);
    cin >> temp; cin >> height;
    cin >> temp; cin >> width;
    getline(cin, temp);

    char c;
    bool passable = false;
    vector<int> traversable;
    vector<uint8_t> passableTerrain { '.', 'G', 'S' };
    uint8_t* pMap = new uint8_t[height * width];
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            cin >> c;
            passable = (count(passableTerrain.begin(),
                              passableTerrain.end(), c) > 0);
            pMap[width*i + j] = passable;
            if(passable) {
                traversable.push_back(width*i+j);
            }
        }
    }

    const int TIMES = 100;
    vector<GridCoordinate> tests; // coordinates test suite

    random_device rng;
    uniform_int_distribution<int> uniform(0, traversable.size()-1);
    for (int i = 0; i < TIMES; ++i)
    {
        int u = traversable[uniform(rng)];
        int v = traversable[uniform(rng)];
        tests.push_back(GridCoordinate_make(u % width, u / width));
        tests.push_back(GridCoordinate_make(v % width, v / width));
    }

    vector<pair<path_finder_func_t, string>> pathFinders;
    pathFinders.push_back(make_pair(BFSFindPath, "BFS"));
    pathFinders.push_back(make_pair(BFSFindPathDiag, "BFSDiag"));
    pathFinders.push_back(make_pair(AStarFindPath, "AStar"));
    pathFinders.push_back(make_pair(AStarFindPathNoTie, "AStarNoTie"));
    pathFinders.push_back(make_pair(AStarFindPathDiag, "AStarDiag"));
    pathFinders.push_back(make_pair(AStarFindPathNoTieDiag, "AStarNoTieDiag"));
    pathFinders.push_back(make_pair(DijkstraFindPath, "Dijkstra"));
    pathFinders.push_back(make_pair(AStarFindPathLandmarks, "AStarLandmarks"));
    pathFinders.push_back(make_pair(AStarFindPathLandmarksDiag, "AStarLandmarksDiag"));

    for (auto& p: pathFinders)
    {
        long long total = 0;
        clock_t start = clock();
        for (int i = 0; i < TIMES; ++i)
        {
            auto u = tests[2*i];
            auto v = tests[2*i+1];
            p.first(u, v, pMap, width, height, NULL, 0);
            total += g_exploredNodes;
        }
        double dur = clock() - start;
        printf("%24s -- avg time: %-3.3fms\tavg nodes: %8lld\n",
            p.second.c_str(),
            double(dur/CLOCKS_PER_SEC) * 1000 / TIMES,
            total / TIMES);
    }

    delete[] pMap;
    return 0;
}
