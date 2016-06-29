#include "region_growing.h"

#define LOAD_FILE

void findDoorCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<pcl::PointXYZ> &centroids) {
    // X-coord width in meters
    const float min_width = .87;
    const float max_width = 1.1;

    // Loop through clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it) {

        // Create cluster from indices
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>(*cloud, (*it).indices));        

        // Calculate OOBB

        // Second check: door width
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(*cluster, min, max);
        float door_width = sqrt(pow(max.x-min.x, 2.) + pow(max.y-min.y, 2.));
        //std::cout << door_width << std::endl;
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
    filter.setLeafSize(.017, .017, .017);
    //std::cout << "Original size: " << cloud->points.size() << std::endl;
    filter.filter(*filtered_cloud);
    std::cout << "Reduced size: " << filtered_cloud->points.size() << std::endl;
}

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2 temp;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(temp, *cloud);

    std::vector<pcl::PointXYZ> centroids;
    processPointCloud(cloud, centroids);
    for (int i = 0;i < centroids.size();++i) {
        geometry_msgs::PointStamped ps;
        geometry_msgs::Point point;
        point.x = centroids[i].x;
        point.y = centroids[i].y;
        point.z = centroids[i].z;
        ps.point = point;

        std_msgs::Header header;
        header.seq = header_seq++;
        header.frame_id = "/world";
        header.stamp = ros::Time::now();
        ps.header = header;
        pub.publish(ps);
    }
}

void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointXYZ> &centroids) {
    // Passthrough filter 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //passthroughFilter(cloud, pass_cloud);

    // Downsample using Voxel Grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    downsample(cloud, filtered_cloud);

    // Calculate normals
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (filtered_cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    // Region growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;

    //Maximum and minimum number of points to classify as a cluster
    // First check: to see if object is large (or takes up a large portion of the laser's view)
    reg.setMinClusterSize(400);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);

    // Number of neighbors to search 
    reg.setNumberOfNeighbours(25);
    reg.setInputCloud(filtered_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(.37);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    //std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

    // Use clusters to find door(s)
    findDoorCentroids(filtered_cloud, clusters, centroids);

    // // Show pointcloud
    if (argc > 2) {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        if (strcmp(argv[2], "-c") == 0) {
            pcl::visualization::CloudViewer viewer ("Cluster viewer");
            viewer.showCloud(colored_cloud);
            while (!viewer.wasStopped())
            {
            }
        } else {
            pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
            viewer.setBackgroundColor(0,0,0);
            viewer.initCameraParameters();
            viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");

            for(std::vector<pcl::PointXYZ>::iterator it = centroids.begin(); it != centroids.end(); ++it) {
                viewer.addSphere(*it, .1, "Sphere" + (it - centroids.begin()));    
            }
            while (!viewer.wasStopped())
            {
                viewer.spinOnce(100);
            }
        }
    }
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
        std::cout << "ERROR: Need filename" << std::endl;
        return (-1);
    }
    // Run tests
    if (strcmp(argv[1], "test") == 0) {
        int num_tests_passed = 0;
        for (int i = 1;i <= 14;++i) {
            std::ostringstream stream;
            stream << "bag_tests/" << i << ".pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(stream.str(), *cloud) == -1) {
                std::cout << "Reading failed for bag: " << i << std::endl;
                return (-1);
            }   
            std::cout << "Loading bag: " << i << ".pcd" << std::endl;
            std::vector<pcl::PointXYZ> centroids;

            // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            // transform.rotate(Eigen::AngleAxisf(1.3, Eigen::Vector3f::UnitZ()));
            // pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            // pcl::transformPointCloud(*cloud, *rotated_cloud, transform);
            processPointCloud(cloud, centroids);
            num_tests_passed += (centroids.size() > 0) ? 1 : 0;     
        }
        std::cout << "PASSED " << num_tests_passed << " TESTS OUT OF 14" << std::endl;
    } else if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    } else {
        std::vector<pcl::PointXYZ> centroids;
        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        //     transform.rotate(Eigen::AngleAxisf(1.3, Eigen::Vector3f::UnitZ()));
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ>());
        //     pcl::transformPointCloud(*cloud, *rotated_cloud, transform);
            processPointCloud(cloud, centroids);
    }    

#else
    // Run in realtime through ROS
    ros::Subscriber sub = n.subscribe("cloud", 1, cloudCB);
    pub = n.advertise<geometry_msgs::PointStamped>("door", 1);

    ros::spin();

#endif

    return (0);

    // TODO: smoothing (robotica.unileon.es)
}