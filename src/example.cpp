#include <ros/ros.h>
//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
	// Container for original and filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// perform the acutal filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(.1, .1, .1);
	sor.filter(cloud_filtered);

	// Publish cloud
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);
	pub.publish(output);
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "my_pcl_tutorial");
	ros::NodeHandle n;

	// Create subscriber for the input point cloud
	ros::Subscriber sub = n.subscribe("cloud", 1, cloud_cb);

	// Creat publisher for output point cloud
	pub = n.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

	ros::spin();
}