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
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_types.hpp>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;

ros::Publisher pub;
Eigen::Vector2f vel(0,0);

void getYaw(float& Yaw, Eigen::Matrix4f &mat)
{
    if (mat(0,0) == 1.0f) {
        Yaw = atan2f(mat(0,2), mat(2,3));
    } else if (mat(0,0) == -1.0f) {
        Yaw = atan2f(mat(0,2), mat(2,3));
    } else {
        Yaw = atan2(-mat(2,0),mat(0,0));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_pcl");
    ros::NodeHandle n;
    
    std::vector<cloud_t::Ptr> cumulative_clouds;
    cloud_t::Ptr cloud_cumulative (new cloud_t);
    cumulative_clouds.push_back(cloud_cumulative);

    cloud_t::Ptr cloud_no_icp (new cloud_t);

    Eigen::Matrix4f matrix_cumulative = Eigen::Matrix4f::Identity();

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
        cloud_t::Ptr cloud_in (new cloud_t);
        cloud_cumulative = cumulative_clouds.back();
        std::cout << cloud_cumulative->points.size() << std::endl;

        if (idx == 50) {
            //break;
        }

        // Load PCD
        cloud_t::Ptr cloud (new cloud_t);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(paths[idx].string(), *cloud) == -1) {
            std::cout << "Cloud reading failed " << paths[idx] << std::endl;
            return (-1);
        }
        std::cout << paths[idx] << std::endl;
        
        *cloud_no_icp += *cloud;
        
        // Chop off close points by distance        
        cloud_t::iterator it = cloud->begin();
        while(it != cloud->end()){
            if (((it->x * it->x) + (it->y * it->y) + (it->z * it->z) < .3)) {// || (it->z < -.12)) {
                it = cloud->erase(it);
            } else { 
                ++it;
            }
        } 

        // Check if we didn't erase whole cloud
        if (cloud->points.size() < 20) {
            continue;
        }

        pcl::transformPointCloud(*cloud, *cloud_in, matrix_cumulative);

        if (idx == 0) {
            *cloud_cumulative = *cloud_in;
            continue;
        }   

        // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // Settings        
        // Set the max correspondence distance
        icp.setMaxCorrespondenceDistance(.15 + 25*(pow(vel(0), 2.) + pow(vel(1), 2.)));
        std::cout << "Corr dist: " << .15 + 25*(pow(vel(0), 2.) + pow(vel(1), 2.)) << std::endl;
        // Set the maximum number of ite rations (criterion 1)
        icp.setMaximumIterations(150);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon(1e-10);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon(1e-14);

        // Align
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_cumulative);
        cloud_t::Ptr Final(new cloud_t);
        icp.align(*Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        std::cout << "Velocity: " << vel(0) << ", " << vel(1) << std::endl;
        
        if (!icp.hasConverged()) {
            vel(0) = 0;
            vel(1) = 0;
            cumulative_clouds.push_back(cloud_in);
            continue;
        }

        Eigen::Matrix4f finalT = icp.getFinalTransformation();

        // Accumulate point cloud
        Eigen::Affine3f finalT_trans(Eigen::Translation3f(finalT(0, 3), finalT(1, 3), 0));
        
        float yaw;
        getYaw(yaw, finalT);
        Eigen::AngleAxis<float> aa(yaw, Eigen::Vector3f(0, 0, 1));
        
        // Reusing 'Final'        
        Eigen::Transform<float, 3, Eigen::Affine> finalT_combined = finalT_trans * aa;
        //pcl::transformPointCloud(*cloud_in, *Final, finalT_combined.matrix());

        *cloud_cumulative += *Final;

        // Change velocity
        if ((fabs(finalT(0, 3)) < .7) && (fabs(finalT(1, 3)) < .7)) {
            vel(0) += finalT(0, 3);
            vel(1) += finalT(1, 3);
            vel(0) /= 2;
            vel(1) /= 2;
            matrix_cumulative = matrix_cumulative * finalT;
        } else {
            vel(0) = 0;
            vel(1) = 0;
            cumulative_clouds.push_back(cloud_in);
            continue;
        }
        
        // Make a guess

        Eigen::Affine3f transform(Eigen::Translation3f(matrix_cumulative(0, 3) + vel(0), 
                                                        matrix_cumulative(1, 3) + vel(1), 0));

        Eigen::Transform<float, 3, Eigen::Affine> combined = transform;// * aa;
        
        matrix_cumulative = combined.matrix();

        std::cout << "Cumulative Translation: " << 
                    matrix_cumulative(0, 3) << ", " << 
                    matrix_cumulative(1, 3) << std::endl;
    }

    // Downsample
    cloud_t::Ptr cloud_downsample (new cloud_t ());

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
    // for (int i = 0; i < cumulative_clouds.size(); i++) {
    //     if (cumulative_clouds[i]->points.size() < 7000) {
    //         //continue;
    //     } 
    //     PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_c (cumulative_clouds[i], rand() % 255, rand() % 255, rand() % 255);
    //     viewer.addPointCloud<pcl::PointXYZ>(cumulative_clouds[i], cloud_c, "cloud" + boost::lexical_cast<std::string>(i));       
    // }
    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cumulative_c (cloud_no_icp, 100, 255, 0);
    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_c (cumulative_clouds[0], 255, 0, 0);

    //viewer.addPointCloud<pcl::PointXYZ>(Final, "Final");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_no_icp, cloud_cumulative_c, "cloud", vp_1);
    viewer.addPointCloud<pcl::PointXYZ>(cumulative_clouds[0], cloud_in_c, "cloud_in", vp_2);
    std::cout << "Collected " << cumulative_clouds.size() << " clouds." << std::endl;
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return (0);
}