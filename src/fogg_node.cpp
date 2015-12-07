
#include "fogg/fogg_node.hpp"

ros::Publisher pub;
Clustering ece;
vector<PCLPointCloudPtr> clusters;
string depth_topic;
float leaf_x, leaf_y, leaf_z, cluster_tolerance, depth_min, depth_max;
int min_cs, max_cs;
int pc_counter = 0;

using namespace std;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    ece.get_euclidean_clusters(input, clusters);
    if (clusters.size() > 0)
    {
        sensor_msgs::PointCloud pc;
        sensor_msgs::ChannelFloat32 ch;
        pc.header.stamp = ros::Time::now();
        pc.header.frame_id = "camera_link";
        pc.header.seq = pc_counter++;
        ch.name = "category";

        for (int i = 0; i < clusters.size(); i++) {
            for (int j = 0; j < clusters[i]->points.size(); j++) {
                geometry_msgs::Point32 p;
                p.x = clusters[i]->points[j].z;
                p.y = -clusters[i]->points[j].x;
                p.z = -clusters[i]->points[j].y;
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
    ros::param::get("~leaf_x", leaf_x);
    ros::param::get("~leaf_y", leaf_y);
    ros::param::get("~leaf_z", leaf_z);
    ros::param::get("~cluster_tolerance", cluster_tolerance);
    ros::param::get("~min_cluster_size", min_cs);
    ros::param::get("~max_cluster_size", max_cs);
    ros::param::get("~depth_topic", depth_topic);
    ros::param::get("~depth_min", depth_min);
    ros::param::get("~depth_max", depth_max);

    ece = Clustering(depth_min, depth_max);
    ece.set_leaf_size(leaf_x, leaf_y, leaf_z);
    ece.set_cluster_tolerance(cluster_tolerance);
    ece.set_min_cluster_size(min_cs);
    ece.set_max_cluster_size(max_cs);

    pub = nh.advertise<sensor_msgs::PointCloud>("fogg_clusters", 1);
    // pub = nh.advertise<nav_msgs::OccupancyGrid>("fogg_grid", 1);
    ros::Subscriber sub = nh.subscribe(depth_topic, 1, cloud_cb);
    ros::spin();
}
