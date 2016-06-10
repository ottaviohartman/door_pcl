#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

void findDoorCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<pcl::PointXYZ> &centroids) {
    // X-coord width (m)
    // TODO: create object aligned axes 
    float min_width = .89;
    float max_width = 1.1;

    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>(*cloud, (*it).indices));
        // First check: to see if object is large (or takes up a large portion of the laser's view)
        if (cluster->points.size() < 10000) {
            continue;
        }

        // Second check: door width
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(*cluster, min, max);
        float door_width = max.x - min.x;
        if ((door_width > min_width) && (door_width < max_width)) {
            Eigen::Matrix<float, 4, 1> centroid;    
            pcl::compute3DCentroid(*cluster, centroid);
            std::cout << door_width << std::endl;
            centroids.push_back(pcl::PointXYZ(centroid(0), centroid(1), centroid(2)));

            std::cout << "Found door with centroid at: " << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << std::endl;
        }   
    }
}

void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud) {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(.3, 3.0);

    filter.filter(*filtered_cloud);
}

void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // Filter size may need to be adjusted  
    filter.setLeafSize(.01, .01, .01);
    std::cout << cloud->points.size() << std::endl;
    filter.filter(*filtered_cloud);
    std::cout << filtered_cloud->points.size() << std::endl;
}

int main(int argc, char** argv)
{
  // Load file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("good3.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    // Passthrough filter 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthroughFilter(cloud, pass_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    downsample(pass_cloud, filtered_cloud);

    // Calculate normals
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (filtered_cloud);
    normal_estimator.setKSearch (40);
    normal_estimator.compute (*normals);

    // Region growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;

    //Maximum and minimum number of points to classify as a cluster
    reg.setMinClusterSize(8000);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);

    // Number of neighbors to search -- Higher value tends to be better
    reg.setNumberOfNeighbours(40);
    reg.setInputCloud(filtered_cloud);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(.3);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;

    for (int i = 0; i < clusters.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr subset(new pcl::PointCloud<pcl::PointXYZ>(*filtered_cloud, clusters[i].indices));
        if (subset->points.size() > 10000) {
            pcl::PointXYZ min;
            pcl::PointXYZ max;
            pcl::getMinMax3D(*subset, min, max);
            std::cout << *subset << std::endl;
            std::cout << "Min: " << min << " Max: " << max << std::endl;
            std::cout << "Width: " << (max.x - min.x) << std::endl;
        }
    }

    std::vector<pcl::PointXYZ> centroids;
    findDoorCentroids(filtered_cloud, clusters, centroids);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
    }
    return (0);

    // TODO: Passthrough filter, downsampling, smoothing (robotica.unileon.es)
}