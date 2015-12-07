
#include "fogg/occupancy.hpp"

void Occupancy::set_occupancy_grid(nav_msgs::OccupancyGrid *og)
{
    this->og = og;
}

void Occupancy::point_to_grid(pcl::PointXYZ& p, int& i, int& j)
{
    i = (int) ((p.x - og->info.origin.position.x) / og->info.resolution);
    j = (int) ((p.y - og->info.origin.position.y) / og->info.resolution);
}

void Occupancy::set(int i, int j, int val)
{
    if (i >= 0 and i < og->info.height and j >= 0 and j < og->info.width)
    {
        og->data[og->info.width * i + j] = val;
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
    if (i >= 0 and i < og->info.height and j >= 0 and j < og->info.width)
    {
        return og->data[og->info.width * i + j];
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
