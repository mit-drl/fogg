#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include "fogg/clustering.hpp"

ros::Publisher pub;
Clustering ece;
vector<PCLPointCloudPtr> clusters;
int j = 0;

using namespace std;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ece.get_euclidean_clusters(input, clusters);
    if (clusters.size() > 0)
    {
        sensor_msgs::PointCloud pc;
        sensor_msgs::ChannelFloat32 ch;
        pc.header.stamp = ros::Time::now();
        pc.header.frame_id = "camera_link";
        pc.header.seq = j++;
        ch.name = "category";

        for (int i = 0; i < clusters.size(); i++) {
            for (int k = 0; k < clusters[i]->points.size(); k++) {
                geometry_msgs::Point32 p;
                p.x = clusters[i]->points[k].x;
                p.y = clusters[i]->points[k].y;
                p.z = clusters[i]->points[k].z;
                pc.points.push_back(p);
                ch.values.push_back(i);
            }
        }
        pc.channels.push_back(ch);
        pub.publish(pc);
    }

}


int main (int argc, char** argv)
{
    ros::init (argc, argv, "fogg_node");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud>("clusters", 1);
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
    ros::spin();
}
