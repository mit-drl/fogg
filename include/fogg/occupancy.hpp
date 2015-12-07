#ifndef OCCUPANCY_HPP
#define OCCUPANCY_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>

#define INVALID -2

class Occupancy
{
    private:
        nav_msgs::OccupancyGrid *og;

    public:
        Occupancy() {};
        ~Occupancy() {};
        void set_occupancy_grid(nav_msgs::OccupancyGrid *og);
        void point_to_grid(pcl::PointXYZ& p, int& i, int& j);
        void set(int i, int j, int val);
        void set(pcl::PointXYZ& p, int val);
        int get(int i, int j);
        int get(pcl::PointXYZ& p);
};

#endif
