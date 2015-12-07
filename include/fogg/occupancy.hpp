#ifndef OCCUPANCY_HPP
#define OCCUPANCY_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_types.h>
#include "fogg/clustering.hpp"

#define INVALID -2

class Occupancy
{
    private:
        float resolution;

    public:
        nav_msgs::OccupancyGrid og;
        Occupancy() {};
        Occupancy(float resolution);
        ~Occupancy() {};
        void setup_og();
        void generate_grid(vector<PCLPointCloudPtr>& clusters);
        void point_to_grid(pcl::PointXYZ& p, int& i, int& j);
        void set(int i, int j, int val);
        void set(pcl::PointXYZ& p, int val);
        int get(int i, int j);
        int get(pcl::PointXYZ& p);
};

#endif
