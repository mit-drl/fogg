#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "fogg/EuclideanClusterExtraction.hpp"

ros::Publisher pub;
EuclideanClusterExtraction ece;
int j = 0;

using namespace std;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    vector<PCLPointCloudPtr> clusters = ece.get_clusters(input);
    if (clusters.size() > 0)
    {
        PCLPointCloudPtr m_cluster;
        int cluster_size = 0;
        for (int i = 0; i < clusters.size(); i++)
        {
            int c_size = clusters[i]->points.size();
            if (c_size > cluster_size)
            {
                m_cluster = clusters[i];
                cluster_size = c_size;
            }
        }
        cout << cluster_size << endl;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*m_cluster, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "camera_depth_frame";
        output.header.seq = j++;
        pub.publish(output);
    }
}


int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_example");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
    ros::spin();
}
