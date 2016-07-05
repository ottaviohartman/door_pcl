#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <boost/filesystem.hpp>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_types.hpp>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

ros::Publisher pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_pcl");
    ros::NodeHandle n;
    
    // if (argc != 3) {
    //     std::cout << "More arguments!" << std::endl;
    //     return -1;
    // }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cumulative (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f matrix_cumulative = Eigen::Matrix4f::Identity();

    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_in) == -1) {
    //     std::cout << "Cloud reading failed 1." << std::endl;
    //     return (-1);
    // }
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_cumulative) == -1) {
    //     std::cout << "Cloud reading failed 2." << std::endl;
    //     return (-1);
    // }
    std::vector<boost::filesystem::path> paths;
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for ( boost::filesystem::directory_iterator itr("."); itr != end_itr; ++itr )
    {
        paths.push_back(itr->path());
    }
    
    std::sort(paths.begin(), paths.end());

    // Loop through file names
    for (size_t idx = 0; idx < paths.size(); idx++) {
        if (!ros::ok()) {
            return -1;
        }
        // Load PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(paths[idx].string(), *cloud) == -1) {
            std::cout << "Cloud reading failed " << paths[idx] << std::endl;
            return (-1);
        }
        std::cout << paths[idx] << std::endl;
      
        // Passthrough
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(1., 6.0);
        filter.filter(*filtered_cloud);
            
        //for (int i = 0; i < 4; i++) {
        //    Eigen::Affine3f transform(Eigen::Translation3f(0.0, 0.0, -.03*i));
        //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::transformPointCloud(*filtered_cloud, *cloud_in, matrix_cumulative);
        
        //    *cloud_in += *cloud_transformed;
        //}
        if (idx == 0) {
            *cloud_cumulative = *cloud_in;
            continue;
        }   
         // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Settings
        
        // Set the max correspondence distance
        icp.setMaxCorrespondenceDistance (0.1);
        // Set the maximum number of ite rations (criterion 1)
        icp.setMaximumIterations (200);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (1e-12);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1e-14);

        // icp.setRANSACOutlierRejectionThreshold(1e-6);
        // icp.setRANSACIterations(100);

        // Align
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_cumulative);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        
        Eigen::Matrix4f finalT = icp.getFinalTransformation();
        // Eigen::Affine3f transform(Eigen::Translation3f(finalT(0,3),finalT(1,3),0));
        matrix_cumulative = matrix_cumulative * finalT;
        // Take away vertical translation
        //matrix_cumulative(2, 3) = 0.0;
        
        std::cout << "Cumulative Translation: " << 
                    matrix_cumulative(0, 3) << ", " << 
                    matrix_cumulative(1, 3) << std::endl;

        // Accumulate point cloud
        *cloud_cumulative += *Final;
        //*cloud_in = *Final;

        //std::cout << cloud_cumulative->points.size() << std::endl;
        // if (idx == 10) {
        //     break;
        // }
    }
    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::VoxelGrid<pcl::PointXYZ> ds_filter;
    ds_filter.setInputCloud(cloud_cumulative);

    // Filter size may need to be adjusted  
    ds_filter.setLeafSize(.03, .03, .03);
    //std::cout << "Original size: " << cloud->points.size() << std::endl;
    ds_filter.filter(*cloud_downsample);

    *cloud_cumulative = *cloud_downsample;

    // Viewer
    pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
    viewer.setBackgroundColor(0,0,0);
    viewer.initCameraParameters();
    // int vp_1, vp_2;
    // viewer.createViewPort(0.0, 0, .5, 1.0, vp_1);
    // viewer.createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cumulative_c (cloud_cumulative, 100, 255, 0);
    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_c (cloud_in, 255, 0, 0);

    //viewer.addPointCloud<pcl::PointXYZ>(Final, "Final");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_cumulative, cloud_cumulative_c, "cloud");
    //viewer.addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_c, "cloud_in");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return (0);
}