
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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

    for (int i = 0; i < clusters.size(); i++)
    {
        vector<cv::Point> cv_pts;
        for (int j = 0; j < clusters[i]->points.size(); j++)
        {
            cv::Point p;
            p.x = clusters[i]->points[j].x;
            p.y = clusters[i]->points[j].y;
            cv_pts.push_back(p);
            set(clusters[i]->points[j], 100);
        }
        cv::fitEllipse(cv_pts);
    }

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
