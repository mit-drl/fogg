#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloudPtr;
typedef pcl::ConditionAnd<pcl::PointXYZ>::Ptr PCLConditionPtr;
typedef pcl::FieldComparison<pcl::PointXYZ>::ConstPtr PCLFieldCompConstPtr;

class Clustering {

    private:
        PCLConditionPtr range_cond;
        PCLFieldCompConstPtr min_comp, max_comp;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        float depth_min, depth_max;

    public:
        PCLPointCloudPtr cloud, cloud_f, cloud_filtered;
        Clustering(float depth_min, float depth_max);
        Clustering() {};
        ~Clustering() {};

        void set_leaf_size(float lx, float ly, float lz);
        void set_cluster_tolerance(float d);
        void set_min_cluster_size(int size);
        void set_max_cluster_size(int size);
        void filter_depth(PCLPointCloudPtr& cloud, PCLPointCloudPtr& cloud_f);
        void get_euclidean_clusters(
                const sensor_msgs::PointCloud2ConstPtr& input,
                vector<PCLPointCloudPtr>& clusters);
};

#endif
