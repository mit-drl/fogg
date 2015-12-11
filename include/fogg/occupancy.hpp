#ifndef OCCUPANCY_HPP
#define OCCUPANCY_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "fogg/clustering.hpp"

#define INVALID -2

typedef pcl::ModelCoefficients::Ptr PCLModelPtr;
typedef pcl::search::KdTree<pcl::PointXYZ>::Ptr PCLKdTreePtr;

class Occupancy
{
    private:
        nav_msgs::OccupancyGrid og;
        float resolution, width, height;
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

    public:
        Occupancy() {};
        Occupancy(float resolution, float width, float height);
        ~Occupancy() {};
        void generate_grid(vector<PCLPointCloudPtr>& clusters);
        void point_to_grid(pcl::PointXYZ& p, int& i, int& j);
        void grid_to_point(int i, int j, pcl::PointXYZ& p);
        bool inside_rectangle(int i, int j, cv::RotatedRect& rect);
        bool inside_rectangle(pcl::PointXYZ& p, cv::RotatedRect& rect);
        float triangle_area(cv::Point2f pts[3]);
        void set(int i, int j, int val);
        void set(pcl::PointXYZ& p, int val);
        int get(int i, int j);
        int get(pcl::PointXYZ& p);
        nav_msgs::OccupancyGrid *get_grid();
};

#endif
