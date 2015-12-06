#ifndef EUCLIDEAN_CLUSTER_EXTRACTION_HPP
#define EUCLIDEAN_CLUSTER_EXTRACTION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloudPtr;

class EuclideanClusterExtraction {

    private:
        PCLPointCloudPtr cloud, cloud_f, cloud_filtered,
            cloud_plane;
        pcl::PointIndices::Ptr inliers;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    public:
        EuclideanClusterExtraction();
        ~EuclideanClusterExtraction() {};

        vector<PCLPointCloudPtr> get_clusters(
                const sensor_msgs::PointCloud2ConstPtr& input);
};

#endif
