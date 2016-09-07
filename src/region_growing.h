#include <iostream>
#include <sstream>
#include <vector>
#include <utility>
#include <algorithm>
#include <ros/ros.h>
#include <ros/callback_queue.h>
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

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
typedef Eigen::Vector3f vec3;

void findDoorCentroids(const cloud_t::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<pcl::PointXYZ> &centroids);
void processPointCloud(const cloud_t::Ptr &cloud, std::vector<point_t> &centroids);
void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void showPointCloudColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
void showPointCloudSphere(const cloud_t &cloud, std::vector<point_t> &centroids);
void downsample(const cloud_t::Ptr &cloud, cloud_t::Ptr &filtered_cloud);
void publishDoor(point_t centroid);

int header_seq = 0;
ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

struct doorPoint {
    int freq;
    point_t center;
};

std::vector<doorPoint> curr_doors;
