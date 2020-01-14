#include "CSF.h"
#include "Vec3.h"
#include "cloth.h"
#include "Rasterization.h"
#include <fstream>
#include <iostream>
CSF::CSF(int index) {
    params.bSloopSmooth     = true;
    params.time_step        = 0.65;
    params.class_threshold  = 0.5;
    params.cloth_resolution = 1;
    params.rigidness        = 3;
    params.interations      = 500;

    this->index = index;
}

CSF::CSF() {
	params.bSloopSmooth = true;
	params.time_step = 0.65;
	params.class_threshold = 0.5;
	params.cloth_resolution = 1;
	params.rigidness = 3;
	params.interations = 500;
	this->index = 0;
}

CSF::~CSF()
{}

void CSF::setPointCloud(std::vector<csf::Point> points) {
    point_cloud.resize(points.size());

    int pointCount = static_cast<int>(points.size());
    //#pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x          = points[i].x;
        las.y          = -points[i].z;
        las.z          = points[i].y;
        point_cloud[i] = las;
    }
}

void CSF::setPointCloud(double *points, int rows) {
	#define A(i, j) points[i + j * rows]

    for (int i = 0; i < rows; i++) {
        csf::Point p;
        p.x = A(i, 0);
        p.y = -A(i, 2);
        p.z = A(i, 1);
        point_cloud.push_back(p);
    }
}

void CSF::setPointCloud(csf::PointCloud& pc) {
    point_cloud.resize(pc.size());
    int pointCount = static_cast<int>(pc.size());
    //#pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x          = pc[i].x;
        las.y          = -pc[i].z;
        las.z          = pc[i].y;
        point_cloud[i] = las;
    }
}


void CSF::do_filtering(std::vector<int>& groundIndexes,
                        std::vector<int>& offGroundIndexes,
                        bool              exportCloth) {
    // Terrain
    std::cout << "[" << this->index << "] Configuring terrain..." << std::endl;
    csf::Point bbMin, bbMax;
    point_cloud.computeBoundingBox(bbMin, bbMax);//找到点云三维坐标的最大值和最小值
    std::printf("bbmin:[%f,%f,%f]\n",bbMin.u[0],bbMin.u[1],bbMin.u[2]);
    std::printf("bbMax:[%f,%f,%f]\n",bbMax.u[0],bbMax.u[1],bbMax.u[2]);

    double cloth_y_height = 0.05;
    
    int  clothbuffer_d = 2;
    Vec3 origin_pos(
        bbMin.x - clothbuffer_d *params.cloth_resolution,
        bbMax.y + cloth_y_height,//保证高于高程最大的值
        bbMin.z - clothbuffer_d *params.cloth_resolution
    );
    
    std::printf("origin_pos:[%f,%f,%f]\n",origin_pos.f[0],origin_pos.f[1],origin_pos.f[2]);
    int width_num = static_cast<int>(
        std::floor((bbMax.x - bbMin.x) / params.cloth_resolution)
    ) + 2 * clothbuffer_d;

    int height_num = static_cast<int>(
        std::floor((bbMax.z - bbMin.z) / params.cloth_resolution)
    ) + 2 * clothbuffer_d;   
    
    std::cout << "[" << this->index << "] Configuring cloth..." << std::endl;
    std::cout << "[" << this->index << "]  - width: " << width_num << " "
        << "height: " << height_num << std::endl;
        
    cloth csf_cloth(
    origin_pos,
    width_num,
    height_num,
    params.cloth_resolution,
    params.cloth_resolution,
    0.3,
    9999,
    params.rigidness,
    params.time_step
    );
    
    std::printf("[%d]Rasterizing...\n",this->index);
    Rasterization::RasterTerrian(csf_cloth, point_cloud, csf_cloth.getHeightvals());
    double time_step2 = params.time_step * params.time_step;
    double gravity = 0.2;
    std::printf("[%d]Simulating...\n",this->index);
    csf_cloth.addForce(Vec3(0, -gravity, 0) * time_step2);
    for (int i = 0; i < params.interations; i++)
    {
        double maxDiff = csf_cloth.timeStep();
        std::printf("    interation[%d] ... max diff:%f\n",i,maxDiff);
        csf_cloth.terrCollision();
        if(maxDiff != 0 && maxDiff < 0.005)
        {
            break;
        }
    }
    if(params.bSloopSmooth)
    {
        std::printf("[%d] - post handle...\n",this->index);
        csf_cloth.movableFilter();
    }
    if(exportCloth)     csf_cloth.saveToFile();
    calCloud2CloudDist(csf_cloth, point_cloud, groundIndexes, offGroundIndexes);
}

void CSF::calCloud2CloudDist(cloth          & cloth_in,
                            csf::PointCloud & pc,
                            std::vector<int>& groundIndexes,
                            std::vector<int>& offGroundIndexes)
{
    groundIndexes.resize(0);
    offGroundIndexes.resize(0);
    for (int i = 0; i < pc.size(); i++)
    {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;
        
        double deltaX = pc_x - cloth_in.origin_pos.f[0];
        double deltaZ = pc_z - cloth_in.origin_pos.f[2];
        
        int col0 = int(deltaX / cloth_in.step_x);
        int row0 = int(deltaZ / cloth_in.step_y);
        int col1 = col0 + 1;
        int row1 = row0;
        int col2 = col0 + 1;
        int row2 = row0 + 1;
        int col3 = col0;
        int row3 = row0 + 1;
        
        double subdeltaX = (deltaX - col0 * cloth_in.step_x) / cloth_in.step_x;
        double subdeltaZ = (deltaZ - row0 * cloth_in.step_y) / cloth_in.step_y;

        double fxy= 
                cloth_in.getParticle(col0, row0)->pos.f[1] * (1 - subdeltaX) * (1 - subdeltaZ) +
                cloth_in.getParticle(col3, row3)->pos.f[1] * (1 - subdeltaX) * subdeltaZ +
                cloth_in.getParticle(col2, row2)->pos.f[1] * subdeltaX * subdeltaZ +
                cloth_in.getParticle(col1, row1)->pos.f[1] * subdeltaX * (1 - subdeltaZ);
        double height_var = fxy - pc[i].y;

        if (std::fabs(height_var) < params.class_threshold)
        {
            groundIndexes.push_back(i);
        } 
        else 
        {
            offGroundIndexes.push_back(i);
        }
    }
    
}



void CSF::savePoints(std::vector<int> grp, std::string path) {
    if (path == "") {
        return;
    }

    std::ofstream f1(path.c_str(), std::ios::out);

    if (!f1)
        return;

    for (std::size_t i = 0; i < grp.size(); i++) {
        f1  << std::fixed << std::setprecision(8)
            << point_cloud[grp[i]].x  << "	"
            << point_cloud[grp[i]].z  << "	"
            << -point_cloud[grp[i]].y << std::endl;
    }

    f1.close();
}