#ifndef _MORPHOLOGICAL_H_
#define _MORPHOLOGICAL_H_
#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
enum MorlOperators
{
    MORPH_OPEN,
    MORPH_CLOSE,
    MORPH_DILATE,
    MORPH_ERODE
};

struct Params {
    double  grid_size;
    int Max_window_size;
    double Terrian_slop;
    double height_diff_threshold_init;
    double height_diff_threshold_max;
};

template <typename PointT>
class MorphologicalFitler
{
private:

public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    
    inline int getMaxWindowSize() const { return (max_window_size_);}
    inline void setMaxWindowSize(int max_window_size) { max_window_size_ = max_window_size; }
    
    inline float getSlope() const { return (slope_); }
    inline void setSlope(float slope) { slope_ = slope; }
    
    inline float getMaxDistance() const { return (max_distance_);}
    inline void setMaxDistance(float max_distance) { max_distance_ = max_distance; }
    
    inline float getInitalDistance() const { return (initial_distance_);}
    inline void setInitalDistance(float initial_distance) { initial_distance_ = initial_distance; }
    
    inline float getCellSize() const { return (cell_size_);}
    inline void setCellSize(float cell_size) { cell_size_ = cell_size;}
    
    inline float getBase() const { return (base_);}
    inline void setBase(float base) { base_ = base; }
    
    inline bool getExponential() const { return (exponential_);}
    inline void setExponential(bool exponential) { exponential_ = exponential;}
    
    inline void setInputCloud(const PointCloudConstPtr &cloud) { input_ = cloud;}
    inline PointCloudConstPtr const getInputCloud() const { return (input_); }
    
    void applyMorphologicalOperator(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                                    float resolution, const int morphological_operator,
                                    pcl::PointCloud<PointT> &cloud_out);                        
    void extract(std::vector<int>& ground);
public:
    MorphologicalFitler();
    ~MorphologicalFitler();
    
protected:
    /** \brief Maximum window size to be used in filtering ground returns. */
    int max_window_size_;

    /** \brief Slope value to be used in computing the height threshold. */
    float slope_;

    /** \brief Maximum height above the parameterized ground surface to be considered a ground return. */
    float max_distance_;

    /** \brief Initial height above the parameterized ground surface to be considered a ground return. */
    float initial_distance_;

    /** \brief Cell size. */
    float cell_size_;

    /** \brief Base to be used in computing progressive window sizes. */
    float base_;

    /** \brief Exponentially grow window sizes? */
    bool exponential_;
    
    PointCloudConstPtr input_;
};

#endif