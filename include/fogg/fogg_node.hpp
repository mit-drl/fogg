#ifndef FOGG_NODE_HPP
#define FOGG_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include "fogg/clustering.hpp"

using namespace std;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

#endif
