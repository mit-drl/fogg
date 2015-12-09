
#include <ros/ros.h>
#include <pcl/common/common.h>
#include "fogg/occupancy.hpp"

Occupancy::Occupancy(float resolution, float width, float height) :
    resolution(resolution),
    width(width), height(height)
{
    og.info.height = (int) (height / resolution);
    og.info.width = (int) (width / resolution);
    og.info.origin.position.x = 0;
    og.info.origin.position.y = -height / 2;
    og.info.origin.position.z = -0.5;
    og.info.resolution = resolution;
    og.header.frame_id = "/camera_link";
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
}

nav_msgs::OccupancyGrid *Occupancy::get_grid()
{
    return &og;
}

void Occupancy::generate_grid(vector<PCLPointCloudPtr>& clusters)
{
    vector<PCLModelPtr> models;
    vector<PCLPointCloudPtr> cyls;
    estimate_cylinders(clusters, models, cyls);
    og.header.stamp = ros::Time::now();
    og.data.clear();
    for (int i = 0; i < og.info.width * og.info.height; i++)
    {
        og.data.push_back(0);
    }

    // for (int i = 0; i < og.info.height; i++)
    // {
    //     for (int j = 0; j < og.info.width; j++)
    //     {
    //         for (int k = 0; k < models.size(); k++)
    //         {
    //             if (inside_cylinder(i, j, models[k]))
    //             {
    //                 set(i, j, 100);
    //             }
    //         }
    //     }
    // }

    for (int i = 0; i < clusters.size(); i++)
    {
        for (int j = 0; j < clusters[i]->points.size(); j++)
        {
            set(clusters[i]->points[j], 100);
        }
    }
}

void Occupancy::estimate_cylinders(vector<PCLPointCloudPtr>& clusters,
        vector<PCLModelPtr>& models, vector<PCLPointCloudPtr>& cyls)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int i = 0; i < clusters.size(); i++)
    {
        // normal estimation
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (
                new pcl::PointCloud<pcl::Normal>);
        PCLKdTreePtr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setInputCloud(clusters[i]);
        ne.setSearchMethod(tree);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        // estimate model parameters
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setInputCloud(clusters[i]);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers, *coefficients);
        models.push_back(coefficients);
        cout << *coefficients << endl;

        // extract the cloud
        PCLPointCloudPtr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
        extract.setInputCloud(clusters[i]);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);
        cyls.push_back(cloud_cylinder);
    }
}

bool Occupancy::inside_cylinder(int i, int j, PCLModelPtr& model)
{
    pcl::PointXYZ p;
    grid_to_point(i, j, p);
    return inside_cylinder(p, model);
}

bool Occupancy::inside_cylinder(pcl::PointXYZ& p, PCLModelPtr& model)
{
    float dx2 = pow(p.x - model->values[0], 2.0);
    float dy2 = pow(p.y - 10 * model->values[1], 2.0);
    return sqrt(dx2 + dy2) < model->values[2];
}

void Occupancy::point_to_grid(pcl::PointXYZ& p, int& i, int& j)
{
    i = (int) ((-p.x - og.info.origin.position.y) / og.info.resolution);
    j = (int) ((p.z - og.info.origin.position.x) / og.info.resolution);
}

void Occupancy::grid_to_point(int i, int j, pcl::PointXYZ& p)
{
    p.y = j * og.info.resolution + og.info.origin.position.x;
    p.x = -(i * og.info.resolution + og.info.origin.position.y);
}

void Occupancy::set(int i, int j, int val)
{
    if (i >= 0 and i < og.info.height and j >= 0 and j < og.info.width)
    {
        og.data[og.info.width * i + j] = val;
    }
}

void Occupancy::set(pcl::PointXYZ& p, int val)
{
    int i, j;
    point_to_grid(p, i, j);
    set(i, j, val);
}

int Occupancy::get(int i, int j)
{
    if (i >= 0 and i < og.info.height and j >= 0 and j < og.info.width)
    {
        return og.data[og.info.width * i + j];
    }
    else
    {
        return INVALID;
    }
}

int Occupancy::get(pcl::PointXYZ& p)
{
    int i, j;
    point_to_grid(p, i, j);
    return get(i, j);
}
