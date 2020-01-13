#include "morphological.h"
template <typename PointT>
MorphologicalFitler<PointT>::MorphologicalFitler():
    max_window_size_(33),
    slope_(0.7f),
    max_distance_(10.0f),
    initial_distance_(0.15f),
    cell_size_(1.0f),
    base_(2.0f),
    exponential_(true)
{

}

template <typename PointT>
MorphologicalFitler<PointT>::~MorphologicalFitler()
{

}

template <typename PointT>
void MorphologicalFitler<PointT>::applyMorphologicalOperator(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                                                            float resolution, const int morphological_operator,
                                                            pcl::PointCloud<PointT> &cloud_out)
{
    if(cloud_in -> empty())  return;
    
    pcl::copyPointCloud<PointT>(*cloud_in,cloud_out);
    pcl::octree::OctreePointCloudSearch<PointT> tree(resolution);//建立八叉树进行搜寻
    
    tree.setInputCloud(cloud_in);
    tree.addPointsFromInputCloud();
    
    float half_res = resolution / 2.0f;
    
    switch(morphological_operator)
    {
        case MORPH_DILATE:
        case MORPH_ERODE:
        {
            for (size_t p_idx = 0; p_idx < cloud_in->size(); ++p_idx)
            {
                Eigen::Vector3f bbox_min, bbox_max;
                std::vector<int> pt_indices;
                float minx = cloud_in->points[p_idx].x - half_res;
                float miny = cloud_in->points[p_idx].y - half_res;
                float minz = -std::numeric_limits<float>::max();
                
                float maxx = cloud_in->points[p_idx].x + half_res;
                float maxy = cloud_in->points[p_idx].y + half_res;
                float maxz = std::numeric_limits<float>::max();
                
                bbox_min = Eigen::Vector3f(minx, miny,minz);
                bbox_max = Eigen::Vector3f(maxx, maxy,maxz);
                tree.boxSearch(bbox_min,bbox_max,pt_indices);// 寻找在这个范围内的所有点
                
                if(pt_indices.size() > 0)
                {
                    Eigen::Vector4f min_pt, max_pt;
                    pcl::getMinMax3D(*cloud_in, pt_indices, min_pt, max_pt);
                    
                    switch(morphological_operator)
                    {
                        case MORPH_DILATE: cloud_out.points[p_idx].z = max_pt.z(); break;
                        case MORPH_ERODE:  cloud_out.points[p_idx].z = min_pt.z(); break;
                    }
                }
            }
            break;
        }
        case MORPH_OPEN:
        case MORPH_CLOSE:
        {
            pcl::PointCloud<PointT> cloud_temp;
            pcl::copyPointCloud<PointT,PointT> (*cloud_in,cloud_temp);
            for (size_t p_idx = 0; p_idx < cloud_in->size(); ++p_idx)
            {
                Eigen::Vector3f bbox_min, bbox_max;
                std::vector<int> pt_indices;
                float minx = cloud_in->points[p_idx].x - half_res;
                float miny = cloud_in->points[p_idx].y - half_res;
                float minz = -std::numeric_limits<float>::max();
                
                float maxx = cloud_in->points[p_idx].x + half_res;
                float maxy = cloud_in->points[p_idx].y + half_res;
                float maxz = std::numeric_limits<float>::max();
                
                bbox_min = Eigen::Vector3f(minx, miny,minz);
                bbox_max = Eigen::Vector3f(maxx, maxy,maxz);
                tree.boxSearch(bbox_min,bbox_max,pt_indices);// 寻找在这个范围内的所有点
                
                if(pt_indices.size() > 0)
                {
                    Eigen::Vector4f min_pt, max_pt;
                    pcl::getMinMax3D(*cloud_in, pt_indices, min_pt, max_pt);
                    
                    switch(morphological_operator)
                    {
                        case MORPH_CLOSE: cloud_out.points[p_idx].z = max_pt.z(); break;
                        case MORPH_OPEN:  cloud_out.points[p_idx].z = min_pt.z(); break;
                    }
                }
            }
            
            cloud_temp.swap(cloud_out);
            for (size_t p_idx = 0; p_idx < cloud_temp.size(); ++p_idx)
            {
                Eigen::Vector3f bbox_min, bbox_max;
                std::vector<int> pt_indices;
                float minx = cloud_temp.points[p_idx].x - half_res;
                float miny = cloud_temp.points[p_idx].y - half_res;
                float minz = -std::numeric_limits<float>::max();
                
                float maxx = cloud_temp.points[p_idx].x + half_res;
                float maxy = cloud_temp.points[p_idx].y + half_res;
                float maxz = std::numeric_limits<float>::max();
                
                bbox_min = Eigen::Vector3f(minx, miny,minz);
                bbox_max = Eigen::Vector3f(maxx, maxy,maxz);
                tree.boxSearch(bbox_min,bbox_max,pt_indices);// 寻找在这个范围内的所有点
                
                if(pt_indices.size() > 0)
                {
                    Eigen::Vector4f min_pt, max_pt;
                    pcl::getMinMax3D(cloud_temp, pt_indices, min_pt, max_pt);
                    
                    switch(morphological_operator)
                    {
                        case MORPH_OPEN:   cloud_out.points[p_idx].z = max_pt.z(); break;
                        case MORPH_CLOSE:  cloud_out.points[p_idx].z = min_pt.z(); break;
                    }
                }
            }            
            break;
        }
        default:
        {
            std:: cerr << "Morphological operator is not supported!\n" << std::endl;
        }
    }
    return ;
}

template <typename PointT> void 
MorphologicalFitler<PointT>::extract(std::vector<int>& ground)
{
    if( input_->points.empty() )
    {
        return;
    }
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;
    int iteration = 0;
    float window_size = 0.0f;
    float height_threshold = 0.0f;
    while( window_size < max_window_size_ )
    {//先根据窗口模型函数分别计算不同迭代次数下的窗口大小，和相应的高度阈值
        if(exponential_)  window_size = cell_size_ * (2.0f * std::pow(base_, iteration) + 1.0f);
        else  window_size = cell_size_ * (2.0f * (iteration + 1)*base_ + 1.0f);
        
        if(iteration == 0)  height_threshold = initial_distance_;
        else height_threshold = slope_* (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_;
        
        if(height_threshold > max_distance_)  height_threshold = max_distance_;
        
        window_sizes.push_back(window_size);
        height_thresholds.push_back(height_threshold);
        
        iteration++;
    }
    for(int i = 0; i < input_ ->points.size(); i++)
    {
        ground.push_back(i);
    }
    // Progressively filter ground returns using morphological open
    for (int i = 0; i < window_sizes.size(); ++i)
    {
        std::printf("Iteration %d (height threshold = %f, window size = %f)...", i,height_thresholds[i],window_sizes[i]);
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud<PointT,PointT>(*input_, ground, *cloud);
        //创建一个新的点云来存储滤波后的数据，实施数学形态法滤波，进行开运算在当前窗口大小
        typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
        applyMorphologicalOperator(cloud,window_sizes[i], MORPH_OPEN, *cloud_f);
        
        std::vector<int> pt_indices;
        for (size_t p_idx = 0; p_idx < ground.size (); ++p_idx)
        {
            float diff = cloud->points[p_idx].z - cloud_f->points[p_idx].z;
            if(diff < height_thresholds[i])  pt_indices.push_back(ground[p_idx]);
        }
        
        ground.swap(pt_indices);
        std::printf("ground now has %zd points\n", ground.size ());
    }
}


template class MorphologicalFitler<pcl::PointXYZ>;//实例化模板类
template class MorphologicalFitler<pcl::PointXYZL>;//实例化模板类