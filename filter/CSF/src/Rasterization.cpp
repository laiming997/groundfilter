#include "Rasterization.h"
#include <queue>

void Rasterization::RasterTerrian(cloth          & cloth,
                                csf::PointCloud& pc,
                                std::vector<double> & heightVal) 
{
    for(int i = 0; i < pc.size(); i++)
    {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;
        
        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int    col    = int(deltaX / cloth.step_x + 0.5);
        int    row    = int(deltaZ / cloth.step_y + 0.5);
        
        if(col >= 0 && col >= 0)
        {
            Particle *pt = cloth.getParticle(col, row);
            pt->correspondingLidarPointList.push_back(i);
            double pc2particleDist = SQUARE_DIST(pc_x,pc_z, pt->getPos().f[0], pt->getPos().f[2]);
            
            if(pc2particleDist < pt->tmpDist)
            {
                pt->tmpDist = pc2particleDist;
                pt->nearestPointHeight = pc[i].y;
                pt->nearestPointIndex = i;
            }
        }
    }
    
    heightVal.resize(cloth.getSize());
    for (int i = 0; i < cloth.getSize(); i++)
    {
        Particle *pcur = cloth.getParticle1d(i);
        double   nearestHeight  = pcur->nearestPointHeight;
        
        if(nearestHeight > MIN_INF)
        {
            heightVal[i] = nearestHeight;
        }
        else heightVal[i] = findHeightValByScanline(pcur,cloth);
    }
    
}

double Rasterization::findHeightValByScanline(Particle *p, cloth& cloth)
{
    int xpos = p->pos_x;
    int ypos = p->pos_y;
    
    for (int i = xpos + 1; i < cloth.num_particles_width; i++)
    {
        double currHeight = cloth.getParticle(i,ypos)->nearestPointHeight;
        if(currHeight > MIN_INF)    return currHeight;
    }
    
    for (int i = xpos -1; i >=0; i--)
    {
        double currHeight = cloth.getParticle(i,ypos)->nearestPointHeight;
        if(currHeight > MIN_INF)    return currHeight;
    }
    
    for (int j = ypos -1; j >=0; j--)
    {
        double currHeight = cloth.getParticle(xpos,j)->nearestPointHeight;
        if(currHeight > MIN_INF)    return currHeight;
    }
    
    for (int j = ypos + 1; j <cloth.num_particles_height; j++)
    {
        double currHeight = cloth.getParticle(xpos,j)->nearestPointHeight;
        if(currHeight > MIN_INF) return currHeight;
    }
    
    return findHeightValByNeighbor(p,cloth);
}

double Rasterization::findHeightValByNeighbor(Particle *p, cloth& cloth)
{
    std::queue<Particle *> nqueue;
    std::vector<Particle *> pbacklist;
    int neiborsize = p->neighborsList.size();
    
    for (int i = 0; i < neiborsize; i++)
    {
        p->isVisited = true;
        nqueue.push(p->neighborsList[i]);
    }
    
    while(!nqueue.empty())
    {
        Particle *pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);
        
        if(pneighbor->nearestPointHeight > MIN_INF)
        {
            for (int i = 0; i < pbacklist.size(); i++)   pbacklist[i]->isVisited = false;
                
            while(!nqueue.empty())
            {
                Particle *pp = nqueue.front();
                pp->isVisited = false;
                nqueue.pop();
            }
            return pneighbor->nearestPointHeight;
        }
        else
        {
            int nsize = pneighbor->neighborsList.size();
            for (int i = 0; i < nsize; i++)
            {
                Particle *ptmp = pneighbor->neighborsList[i];
                if(!ptmp->isVisited)
                {   
                    ptmp->isVisited = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }
    return MIN_INF;
}