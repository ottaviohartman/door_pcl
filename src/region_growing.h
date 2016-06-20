#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/moment_of_inertia_estimation.h>

// To be moved to a .h
void findDoorCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<pcl::PointXYZ> &centroids);
void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointXYZ> &centroids);
void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);
void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);

int argc;
int header_seq=0;
char **argv;
ros::Publisher pub;