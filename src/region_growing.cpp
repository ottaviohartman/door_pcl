#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/common/common.h>
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

//#define LOAD_FILE

// To be moved to a .h
void findDoorCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<pcl::PointXYZ> &centroids);
void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointXYZ> &centroids);
void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);
void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);

int argc;
char **argv;
ros::Publisher pub;

void findDoorCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<pcl::PointXYZ> &centroids) {
    // X-coord width (m)
    // TODO: create object aligned axes 
    float min_width = .89;
    float max_width = 1.1;

    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it) {
        // Create cluster from indices
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>(*cloud, (*it).indices));

        // TODO: more robust door checking
        // First check: to see if object is large (or takes up a large portion of the laser's view)
        if (cluster->points.size() < 4000) {
            continue;
        }
        // Second check: door width
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(*cluster, min, max);
        float door_width = max.x - min.x;

        if ((door_width > min_width) && (door_width < max_width)) {
            // Return door centroid
            Eigen::Matrix<float, 4, 1> centroid;    
            pcl::compute3DCentroid(*cluster, centroid);
            centroids.push_back(pcl::PointXYZ(centroid(0), centroid(1), centroid(2)));
            std::cout << "Found door with centroid at: " << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << std::endl;
        }   
    }
}

void passthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud) {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(0., 3.0);
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

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2 temp;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(temp, *cloud);

    std::vector<pcl::PointXYZ> centroids;
    processPointCloud(cloud, centroids);
    geometry_msgs::Point point;
    point.x = centroids[0].x;
    point.y = centroids[0].y;
    point.z = centroids[0].z;
    pub.publish(point);
}

void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointXYZ> &centroids) {
    // Passthrough filter 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthroughFilter(cloud, pass_cloud);

    // Downsample using Voxel Grid
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
    reg.setMinClusterSize(80);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);

    // Number of neighbors to search -- Higher value tends to be better
    reg.setNumberOfNeighbours(40);
    reg.setInputCloud(filtered_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(.3);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

    // Use clusters to find door(s)
    findDoorCentroids(filtered_cloud, clusters, centroids);

    // // Show pointcloud
    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    // if ((argc > 2) && (strcmp(argv[2], "-c") == 0)) {
    //     pcl::visualization::CloudViewer viewer ("Cluster viewer");
    //     viewer.showCloud(colored_cloud);
    //     while (!viewer.wasStopped())
    //     {
    //     }
    // } else {
    //     pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
    //     viewer.setBackgroundColor(0,0,0);
    //     viewer.initCameraParameters();
    //     viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");

    //     for(std::vector<pcl::PointXYZ>::iterator it = centroids.begin(); it != centroids.end(); ++it) {
    //         viewer.addSphere(*it, .1, "Sphere" + (it - centroids.begin()));    
    //     }
    //     while (!viewer.wasStopped())
    //     {
    //         viewer.spinOnce(100);
    //     }
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_pcl");
    ros::NodeHandle n;
    ::argc = argc;
    ::argv = argv;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

#ifdef LOAD_FILE    
    
    if (argc < 2) {
        std::cout << "Need filename";
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    std::vector<pcl::PointXYZ> centroids;
    processPointCloud(cloud, centroids);    

#else

    ros::Subscriber sub = n.subscribe("cloud", 2, cloudCB);
    pub = n.advertise<geometry_msgs::Point>("door", 1);

    ros::spin();

#endif

    return (0);

    // TODO: smoothing (robotica.unileon.es)
}