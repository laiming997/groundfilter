#include <iostream>
#include "ProgressiveMorphological/include/morphological.h"
#include "common/include/FileReader.h"
#include "common/include/Config.h"
#include "CSF/include/CSF.h"

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
int main(int argc, char const *argv[])
{
    if (argc < 2)
    {
		fprintf(stderr, "Usage: %s, Input Filename\n", argv[0]);
		system("PAUSE");
		return -1;
	}	
	FileReader reader(argv[1]);
	pcl::PointCloud<pcl::PointXYZL> cloud_ref;
    pcl::PointCloud<pcl::PointXYZ> Ground_cloud;
    pcl::PointCloud<pcl::PointXYZ> Object_cloud;
    std::tie(cloud_ref, Ground_cloud,Object_cloud)= reader.RetxtReader();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> cloud_filter(new pcl::PointCloud<pcl::PointXYZL>);
	sor.setInputCloud(cloud_ref.makeShared());
	sor.setMeanK(20);
	sor.setStddevMulThresh(3.0);
	sor.filter(*cloud_filter);//先预处理点云去掉一些孤立点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i <cloud_filter->size();i++)
	{
		cloud_in->push_back(pcl::PointXYZ(cloud_filter->points[i].x, cloud_filter->points[i].y, cloud_filter->points[i].z));
	}
	std::string ConfigFile= argv[2];
	Config configSettings(ConfigFile);
	Params param;
	param.grid_size = configSettings.Read("grid_size",0.25);
	param.height_diff_threshold_init = configSettings.Read("height_diff_threshold_init",0.5);
	param.height_diff_threshold_max = configSettings.Read("height_diff_threshold_max",3);
	param.Max_window_size = configSettings.Read("Max_window_size",38);
	param.Terrian_slop = configSettings.Read("Terrian_slop",1.0);//数学形态法的参数
	double angle_threshold =  configSettings.Read("angle_threshold",2.0); // angle in radians
	double distance_threshold = configSettings.Read("distance_threshold",0.2);
	
	printf("grid_size:%lf\n",param.grid_size);
	printf("height_diff_threshold_init:%lf\n",param.height_diff_threshold_init);
	printf("Max_window_size:%d\n",param.Max_window_size);
	printf("Terrian_slop:%lf\n",param.Terrian_slop);
	printf("angle_threshold:%lf\n",angle_threshold);
	printf("distance_threshold:%lf\n",distance_threshold);
	// MorphologicalFitler<pcl::PointXYZ> my_pmf;
	// pcl::PointIndicesPtr ground(new pcl::PointIndices);
	// my_pmf.setInputCloud(cloud_in);
	// my_pmf.setMaxWindowSize(param.Max_window_size);
	// my_pmf.setSlope(param.Terrian_slop);
	// my_pmf.setInitalDistance(0.5f);
	// my_pmf.setMaxDistance(3.0f);
	// my_pmf.extract(ground->indices);
	CSF csf;
	std::vector<csf::Point> input_points;
	for (int i = 0; i < cloud_in->size(); i++)
	{
		csf::Point point;
		point.x = cloud_in->points[i].x;
		point.y = cloud_in->points[i].y;
		point.z = cloud_in->points[i].z;
		input_points.push_back(point);
	}
	csf.setPointCloud(input_points);
	std::vector<int> groundIndexes, offGroundIndexes;
	csf.do_filtering(groundIndexes, offGroundIndexes, true);
    return 0;
}
