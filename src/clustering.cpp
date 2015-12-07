
#include "fogg/clustering.hpp"

Clustering::Clustering(float depth_min, float depth_max) :
    depth_min(depth_min), depth_max(depth_max),
    cloud(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_f(new pcl::PointCloud<pcl::PointXYZ>),
    cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
    tree(new pcl::search::KdTree<pcl::PointXYZ>),
    range_cond(new pcl::ConditionAnd<pcl::PointXYZ>()),
    min_comp(new pcl::FieldComparison<pcl::PointXYZ>("z",
                pcl::ComparisonOps::GT, depth_min)),
    max_comp(new pcl::FieldComparison<pcl::PointXYZ>("z",
                pcl::ComparisonOps::LT, depth_max))
{
    vg.setLeafSize(0.03f, 0.03f, 0.01f);
    ec.setClusterTolerance (0.03);
    ec.setMinClusterSize(400);
    ec.setMaxClusterSize(5000);
    range_cond->addComparison(min_comp);
    range_cond->addComparison(max_comp);
};

void Clustering::set_leaf_size(float lx, float ly, float lz)
{
    vg.setLeafSize(lx, ly, lz);
}

void Clustering::set_cluster_tolerance(float d)
{
    ec.setClusterTolerance(d);
}

void Clustering::set_min_cluster_size(int size)
{
    ec.setMinClusterSize(size);
}

void Clustering::set_max_cluster_size(int size)
{
    ec.setMaxClusterSize(size);
}

void Clustering::filter_depth(PCLPointCloudPtr& cloud,
        PCLPointCloudPtr& cloud_f)
{
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud_f);
}

void Clustering::get_occupancy_grid(vector<PCLPointCloudPtr> clusters,
        nav_msgs::OccupancyGrid& og)
{
    og.info.resolution = 0.2;
    og.info.width = 100;
    og.info.height = 100;
    og.header.stamp = ros::Time::now();
    og.header.frame_id = "camera_link";
}

void Clustering::get_euclidean_clusters(
        const sensor_msgs::PointCloud2ConstPtr& input,
        vector<PCLPointCloudPtr>& clusters)
{
    pcl::fromROSMsg(*input, *cloud);
    filter_depth(cloud, cloud_f);
    vg.setInputCloud(cloud_f);
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
