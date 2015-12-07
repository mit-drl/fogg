
#include "fogg/clustering.hpp"

Clustering::Clustering() :
    cloud(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_f(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
    tree(new pcl::search::KdTree<pcl::PointXYZ>)
{
    vg.setLeafSize(0.03f, 0.03f, 0.01f);
    ec.setClusterTolerance (0.03);
    ec.setMinClusterSize(400);
    ec.setMaxClusterSize(5000);
};

void Clustering::get_euclidean_clusters(
        const sensor_msgs::PointCloud2ConstPtr& input,
        vector<PCLPointCloudPtr>& clusters)
{
    pcl::fromROSMsg(*input, *cloud);
    vg.setInputCloud(cloud);
    vg.filter(*cloud_filtered);

    tree->setInputCloud(cloud_filtered);
    vector<pcl::PointIndices> cluster_indices;
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    if (cluster_indices.size() < clusters.size()) {
        int ds = clusters.size() - cluster_indices.size();
        clusters.erase(clusters.end() - ds, clusters.end());
    }

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

        if (i < clusters.size()) {
            clusters[i] = cloud_cluster;
        } else {
            clusters.push_back(cloud_cluster);
        }
    }
}
