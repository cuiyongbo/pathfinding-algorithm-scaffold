/*
 * scenari_loader.cpp
 * hog
 *
 * Created by Renee Jansen on 5/2/2006
 * copied from https://github.com/nathansttt/hog2
 *
 */

#include <assert.h>
#include <fstream>
#include "scenario_loader.h"

using std::ifstream;
using std::ofstream;

ScenarioLoader::ScenarioLoader(const char* fname)
{
    strncpy(scenName, fname, 1024);
    ifstream sfile(fname,std::ios::in);

    // Check if a version number is given
    float ver;
    string first;
    sfile >> first;
    if(first != "version")
    {
        ver = 0.0;
        sfile.seekg(0,std::ios::beg);
    }
    else
    {
        sfile>>ver;
    }

    int sizeX = 0, sizeY = 0;
    int bucket;
    string map;
    int xs, ys, xg, yg;
    double dist;

    // Read in & store experiments
    if (ver==0.0)
    {
        while(sfile>>bucket>>map>>xs>>ys>>xg>>yg>>dist)
        {
            Experiment exp(xs,ys,xg,yg,bucket,dist,map);
            experiments.push_back(exp);
        }
    }
    else if(ver==1.0)
    {
        while(sfile>>bucket>>map>>sizeX>>sizeY>>xs>>ys>>xg>>yg>>dist)
        {
            Experiment exp(xs,ys,xg,yg,sizeX,sizeY,bucket,dist,map);
            experiments.push_back(exp);
        }
    }
    else
    {
        fprintf(stderr, "Invalid version number.\n");
        assert(0);
    }
}

void ScenarioLoader::Save(const char *fname)
{
    float ver = 1.0;
    ofstream ofile(fname);
    ofile << "version " << ver << std::endl;
    for (unsigned int x = 0; x < experiments.size(); x++)
    {
        ofile<<experiments[x].bucket<<"\t"<<experiments[x].map<<"\t"<<experiments[x].scaleX<<"\t";
        ofile<<experiments[x].scaleY<<"\t"<<experiments[x].startx<<"\t"<<experiments[x].starty<<"\t";
        ofile<<experiments[x].goalx<<"\t"<<experiments[x].goaly<<"\t"<<experiments[x].distance<<std::endl;
    }
}

void ScenarioLoader::AddExperiment(Experiment which)
{
    experiments.push_back(which);
}
