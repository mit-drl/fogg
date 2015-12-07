#ifndef EUCLIDEAN_CLUSTER_EXTRACTION_HPP
#define EUCLIDEAN_CLUSTER_EXTRACTION_HPP

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
#include <vector>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloudPtr;

class Clustering {

    private:
        PCLPointCloudPtr cloud, cloud_f, cloud_filtered;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    public:
        Clustering();
        ~Clustering() {};

        void get_euclidean_clusters(
                const sensor_msgs::PointCloud2ConstPtr& input,
                vector<PCLPointCloudPtr>& clusters);
};

#endif
