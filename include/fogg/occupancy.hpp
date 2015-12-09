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
        void estimate_cylinders(vector<PCLPointCloudPtr>& clusters,
                vector<PCLModelPtr>& models, vector<PCLPointCloudPtr>& cyls);
        void point_to_grid(pcl::PointXYZ& p, int& i, int& j);
        void grid_to_point(int i, int j, pcl::PointXYZ& p);
        bool inside_cylinder(int i, int j, PCLModelPtr& model);
        bool inside_cylinder(pcl::PointXYZ& p, PCLModelPtr& model);
        void set(int i, int j, int val);
        void set(pcl::PointXYZ& p, int val);
        int get(int i, int j);
        int get(pcl::PointXYZ& p);
        nav_msgs::OccupancyGrid *get_grid();
};

#endif
