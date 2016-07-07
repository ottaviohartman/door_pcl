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
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <boost/filesystem.hpp>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_types.hpp>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

ros::Publisher pub;
Eigen::Vector2f vel(0,0);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_icp (new pcl::PointCloud<pcl::PointXYZ>);

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

        if (idx == 25) {
            //break;
        }

        // Load PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(paths[idx].string(), *cloud) == -1) {
            std::cout << "Cloud reading failed " << paths[idx] << std::endl;
            return (-1);
        }
        std::cout << paths[idx] << std::endl;
        
        *cloud_no_icp += *cloud;
        
        // Chop off close points by distance        
        pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();
        while(it != cloud->end()){
            if ((it->x * it->x) + (it->y * it->y) + (it->z * it->z) < .4) {
                it = cloud->erase(it);
            } else { 
                ++it;
            }
        } 
            
        // Transform estimation from last matrix
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>); 

        pcl::transformPointCloud(*cloud, *cloud_in, matrix_cumulative);
        
        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // transform.translate(Eigen::Vector3f(0.0, 0.0, -.1));
        // pcl::transformPointCloud(*cloud2, *cloud_in, transform);

        // *cloud_in += *cloud2;

        if (idx == 0) {
            *cloud_cumulative = *cloud_in;
            continue;
        }   
         // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Settings
        
        // Set the max correspondence distance
        icp.setMaxCorrespondenceDistance (.3);
        // Set the maximum number of ite rations (criterion 1)
        icp.setMaximumIterations (500);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (1e-10);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1e-14);

        // icp.setRANSACOutlierRejectionThreshold(1e-6);
        // icp.setRANSACIterations(100000);

        // Align
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_cumulative);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        std::cout << "Velocity: " << vel(0) << ", " << vel(1) << std::endl;
        if (!icp.hasConverged()) {
            vel(0) = 0;
            vel(1) = 0;
            *cloud_cumulative += *cloud_in;
            continue;
        }
        // Accumulate point cloud
        //Eigen::Affine3f finalT_trans(Eigen::Translation3f(finalT(0, 3), finalT(1, 3), 0));
        // Reusing 'Final'
        //pcl::transformPointCloud(*cloud_in, *Final, finalT_trans.matrix());

        *cloud_cumulative += *Final;

        Eigen::Matrix4f finalT = icp.getFinalTransformation();
        if ((fabs(finalT(0, 3)) < 2) && (fabs(finalT(1, 3)) < 2)) {
            vel(0) += finalT(0, 3);
            vel(1) += finalT(1, 3);
            vel(0) /= 2;
            vel(1) /= 2;
            matrix_cumulative = matrix_cumulative * finalT;
        } else {
            vel(0) = 0;
            vel(1) = 0;
        }
        
        // Make a guess
        
        // matrix_cumulative(0, 3) += vel(0);
        // matrix_cumulative(1, 3) += vel(1);

        Eigen::Affine3f transform(Eigen::Translation3f(matrix_cumulative(0, 3) + vel(0), 
                                                        matrix_cumulative(1, 3) + vel(1), 0));
        matrix_cumulative = transform.matrix();
        
        std::cout << "Cumulative Translation: " << 
                    matrix_cumulative(0, 3) << ", " << 
                    matrix_cumulative(1, 3) << std::endl;
        //std::cout << cloud_cumulative->points.size() << std::endl;
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::VoxelGrid<pcl::PointXYZ> ds_filter;
    ds_filter.setInputCloud(cloud_cumulative);

    // Filter size may need to be adjusted  
    ds_filter.setLeafSize(.025, .025, .025);
    //std::cout << "Original size: " << cloud->points.size() << std::endl;
    //ds_filter.filter(*cloud_downsample);

    //*cloud_cumulative = *cloud_downsample;

    // Viewer
    pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
    viewer.setBackgroundColor(0,0,0);
    viewer.initCameraParameters();
    int vp_1, vp_2;
    viewer.createViewPort(0.0, 0, .5, 1.0, vp_1);
    viewer.createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cumulative_c (cloud_cumulative, 100, 255, 0);
    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_c (cloud_no_icp, 255, 0, 0);

    //viewer.addPointCloud<pcl::PointXYZ>(Final, "Final");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_cumulative, cloud_cumulative_c, "cloud", vp_1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_no_icp, cloud_in_c, "cloud_in", vp_2);

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return (0);
}