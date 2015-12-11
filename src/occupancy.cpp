
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
    og.header.stamp = ros::Time::now();
    og.data.clear();

    for (int i = 0; i < og.info.width * og.info.height; i++)
    {
        og.data.push_back(0);
    }

    for (int i = 0; i < clusters.size(); i++)
    {
        vector<cv::Point2f> cv_pts;
        for (int j = 0; j < clusters[i]->points.size(); j++)
        {
            cv::Point2f p;
            p.x = -clusters[i]->points[j].x;
            p.y = clusters[i]->points[j].z;
            cv_pts.push_back(p);
            set(clusters[i]->points[j], 100);
        }

        // cv::RotatedRect rect = cv::fitEllipse(cv_pts);
        // for (int k = 0; k < og.info.height; k++)
        // {
        //     for (int l = 0; l < og.info.width; l++)
        //     {
        //         if (inside_rectangle(k, l, rect))
        //         {
        //             set(k, l, 100);
        //         }
        //     }
        // }
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

bool Occupancy::inside_rectangle(int i, int j, cv::RotatedRect& rect)
{
    pcl::PointXYZ p;
    grid_to_point(i, j, p);
    return inside_rectangle(p, rect);
}

bool Occupancy::inside_rectangle(pcl::PointXYZ& p_, cv::RotatedRect& rect)
{
    cv::Point2f pts[4];
    rect.points(pts);
    cv::Point2f p;
    p.x = p_.x;
    p.y = p_.y;
    rect.points(pts);
    float s, ra, apd, dpc, cpb, pba;
    cv::Point2f a = pts[0], b = pts[1], c = pts[2], d = pts[3];
    cv::Point2f tapd[] = {a, p, d};
    apd = triangle_area(tapd);
    cv::Point2f tdpc[] = {d, p, c};
    dpc = triangle_area(tdpc);
    cv::Point2f tcpb[] = {c, p, b};
    cpb = triangle_area(tcpb);
    cv::Point2f tpba[] = {p, b, a};
    pba = triangle_area(tpba);
    s = apd + dpc + cpb + pba;
    ra = rect.size.width * rect.size.height;
    return s <= ra;
}

float Occupancy::triangle_area(cv::Point2f pts[3]) {
    float A, B, C;
    cv::Point2f a = pts[0], b = pts[1], c = pts[2];
    A = a.x * (b.y - c.y);
    B = b.x * (c.y - a.y);
    C = c.x * (a.y - b.y);
    return fabsf((A + B + C) / 2.0);
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
