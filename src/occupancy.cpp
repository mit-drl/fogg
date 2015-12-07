
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <tf/tf.h>
#include "fogg/occupancy.hpp"

Occupancy::Occupancy(float resolution) : resolution(resolution)
{
}

void Occupancy::setup_og()
{
    pcl::PointXYZ pmin, pmax;
    og.info.origin.position.x = 0;
    og.info.origin.position.y = -100 * resolution / 2;
    og.info.origin.position.z = -0.5;
    og.info.height = 100;
    og.info.width = 100;
    og.info.resolution = resolution;
    og.header.stamp = ros::Time::now();
    og.header.frame_id = "/camera_link";
    og.data.clear();
    for (int i = 0; i < og.info.width * og.info.height; i++)
    {
        og.data.push_back(0);
    }
}

void Occupancy::generate_grid(vector<PCLPointCloudPtr>& clusters)
{
    setup_og();
    for (int i = 0; i < clusters.size(); i++)
    {
        for (int j = 0; j < clusters[i]->points.size(); j++)
        {
            set(clusters[i]->points[j], 100);
        }
    }
}

void Occupancy::point_to_grid(pcl::PointXYZ& p, int& i, int& j)
{
    j = (int) ((p.z - og.info.origin.position.x) / og.info.resolution);
    i = (int) ((-p.x - og.info.origin.position.y) / og.info.resolution);
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
