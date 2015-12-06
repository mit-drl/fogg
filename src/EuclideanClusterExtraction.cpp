
#include "fogg/EuclideanClusterExtraction.hpp"

EuclideanClusterExtraction::EuclideanClusterExtraction() :
    cloud(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_f(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_plane(new pcl::PointCloud<pcl::PointXYZ>()),
    inliers(new pcl::PointIndices),
    coefficients(new pcl::ModelCoefficients),
    tree(new pcl::search::KdTree<pcl::PointXYZ>)
{
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize(2500);
    ec.setMaxClusterSize(5000);
};

vector<PCLPointCloudPtr> EuclideanClusterExtraction::get_clusters(
        const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg(*input, *cloud);
    vg.setInputCloud(cloud);
    vg.filter(*cloud_filtered);

    int nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 1 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud(cloud_filtered);
    vector<pcl::PointIndices> cluster_indices;
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    vector<PCLPointCloudPtr> clusters;
    for (int i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
            int ind = cluster_indices[i].indices[j];
            cloud_cluster->points.push_back(cloud_filtered->points[ind]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}
