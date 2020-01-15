// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

#include "Rasterization.h"
#include <queue>


double Rasterization::findHeightValByScanline(Particle *p, Cloth& cloth) {
    int xpos = p->pos_x;
    int ypos = p->pos_y;

    for (int i = xpos + 1; i < cloth.num_particles_width; i++) {
        double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    for (int i = xpos - 1; i >= 0; i--) {
        double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    for (int j = ypos - 1; j >= 0; j--) {
        double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    for (int j = ypos + 1; j < cloth.num_particles_height; j++) {
        double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }
    // std::cout<< "findHeightValByScanline:["<<xpos<<","<<ypos<<"]"<<std::endl;
    return findHeightValByNeighbor(p, cloth);
}


double Rasterization::findHeightValByNeighbor(Particle *p, Cloth& cloth) {
    std::queue<Particle *>  nqueue;
    std::vector<Particle *> pbacklist;
    int neiborsize = p->neighborsList.size();
    // std::cout<< "findHeightValByNeighbor:["<<p->pos_x<<","<<p->pos_y<<"]"<<" neiborsize"<<neiborsize<<std::endl;
    for (int i = 0; i < neiborsize; i++) {
        p->isVisited = true;
        nqueue.push(p->neighborsList[i]);
        // std::cout<<"[" <<p->neighborsList[i]->pos_x<<","<<p->neighborsList[i]->pos_y<<"]"<<std::endl;
    }

    // iterate over the nqueue
    // 寻找最近的具有nearestPointHeight > MIN_INF的点，并使isVisited参数复位成false
    while (!nqueue.empty()) {
        Particle *pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);

        if (pneighbor->nearestPointHeight > MIN_INF) {
            for (std::size_t i = 0; i < pbacklist.size(); i++)
                pbacklist[i]->isVisited = false;

            while (!nqueue.empty()) {
                Particle *pp = nqueue.front();
                pp->isVisited = false;
                nqueue.pop();
            }
            // std::cout<<"最终的领点[" <<pneighbor->pos_x<<","<<pneighbor->pos_y<<"]"
            //         <<"最终的领点的点云"<<"["<<pneighbor->getPos().f[0]<<","<<pneighbor->getPos().f[1]<<","<<pneighbor->getPos().f[0]<<"]"
            //         <<"最终的nearestPointHeight："<<pneighbor->nearestPointHeight<<std::endl;
            return pneighbor->nearestPointHeight;
        } else {
            int nsize = pneighbor->neighborsList.size();

            for (int i = 0; i < nsize; i++) {
                Particle *ptmp = pneighbor->neighborsList[i];

                if (!ptmp->isVisited) {
                    ptmp->isVisited = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }

    return MIN_INF;
}

void Rasterization::RasterTerrian(Cloth          & cloth,
                                  csf::PointCloud& pc,
                                  std::vector<double> & heightVal) {
    
    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int    col    = int(deltaX / cloth.step_x );
        int    row    = int(deltaZ / cloth.step_y );

        if ((col >= 0) && (row >= 0)) {
            Particle *pt = cloth.getParticle(col, row);
            pt->correspondingLidarPointList.push_back(i);
            double pc2particleDist = SQUARE_DIST(
                pc_x, pc_z,
                pt->getPos().f[0],
                pt->getPos().f[2]
            );

            if (pc2particleDist < pt->tmpDist) {
                pt->tmpDist            = pc2particleDist;
                pt->nearestPointHeight = pc[i].y;
                pt->nearestPointIndex  = i;
            }
        }
    }
    heightVal.resize(cloth.getSize());

    //#pragma omp parallel for
    for (int i = 0; i < cloth.getSize(); i++) {
        Particle *pcur          = cloth.getParticle1d(i);
        double    nearestHeight = pcur->nearestPointHeight;

        if (nearestHeight > MIN_INF) {
            heightVal[i] = nearestHeight;
        } else {
            heightVal[i] = findHeightValByScanline(pcur, cloth);
        }
    }
}
