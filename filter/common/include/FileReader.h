#ifndef _FILEREADER_H_
#define _FILEREADER_H_
#include <tuple>
#include <fstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
class FileReader
{
private:
    const char* file_path;
public:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> txtReader();
    std::tuple<pcl::PointCloud<pcl::PointXYZL> , pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ>> RetxtReader();
    bool LasReader();
public:
    FileReader();
    FileReader(const char* input_file_name);
    ~FileReader();
};




#endif