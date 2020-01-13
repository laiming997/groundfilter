#ifndef _CSF_H_
#define _CSF_H_
#include <vector>
#include <string>
#include <iostream>
#include "point_cloud.h"

namespace csf{
struct Params {
    // refer to the website:http://ramm.bnu.edu.cn/projects/CSF/ for the setting of these paramters
    bool bSloopSmooth;
    double time_step;
    double class_threshold;
    double cloth_resolution;
    int rigidness;
    int interations;
};
}

class CSF
{
private:
    csf::PointCloud point_cloud;
public:
    CSF(int index);
    CSF();
    ~CSF();
    
    // set pointcloud from vector
    void setPointCloud(std::vector<csf::Point> points);
    // set point cloud from a one-dimentional array. it defines a N*3 point cloud by the given rows.
    void setPointCloud(double *points, int rows);
    // PointCloud set pointcloud
    void setPointCloud(csf::PointCloud& pc);
    
    inline csf::PointCloud& getPointCloud() {
        return point_cloud;
    }

    inline const csf::PointCloud& getPointCloud() const {
        return point_cloud;
    }
    
    // save points to file
    void savePoints(std::vector<int> grp, std::string path);
    
    // get size of pointcloud
    std::size_t size() {
        return point_cloud.size();
    }
    

    // The results are index of ground points in the original
    // pointcloud
    void do_filtering(std::vector<int>& groundIndexes,
                    std::vector<int>& offGroundIndexes,
                    bool              exportCloth = false);
public:
    csf::Params params;
    int index;
};


#endif