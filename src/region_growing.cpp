#include "region_growing.h"

#define LOAD_FILE

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
typedef Eigen::Vector3f vec3;

inline float dist(point_t a, point_t b) {
    vec3 a_(a.x, a.y, a.z);
    vec3 b_(b.x, b.y, b.z);
    return (a_ - b_).norm();
}

struct cmpPoints {
    bool operator()(const point_t& a, const point_t& b) const {
        return a.x < b.x;
    }
};

void findDoorCentroids(const cloud_t::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<point_t> &centroids) {
    // X-coord width in meters
    const float min_width = .87;
    const float max_width = 1.1;

    // Loop through clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it) {

        // Create cluster from indices
        cloud_t::Ptr cluster(new cloud_t(*cloud, (*it).indices));        

        // Calculate OOBB

        // Second check: door width
        point_t min;
        point_t max;
        pcl::getMinMax3D(*cluster, min, max);
        float door_width = sqrt(pow(max.x-min.x, 2.) + pow(max.y-min.y, 2.));
        //std::cout << door_width << std::endl;
        if ((door_width > min_width) && (door_width < max_width)) {
            // Return door centroid
            Eigen::Matrix<float, 4, 1> centroid;    
            pcl::compute3DCentroid(*cluster, centroid);
            centroids.push_back(point_t(centroid(0), centroid(1), centroid(2)));
            std::cout << "Found door with centroid at: " << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << std::endl;
        }   
    }
}

void passthroughFilter(const cloud_t::Ptr &cloud, cloud_t::Ptr &filtered_cloud) {
    pcl::PassThrough<point_t> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(0., 3.0);
    filter.filter(*filtered_cloud);
}

void downsample(const cloud_t::Ptr &cloud, cloud_t::Ptr &filtered_cloud) {
    pcl::VoxelGrid<point_t> filter;
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
    cloud_t::Ptr cloud(new cloud_t);
    pcl::fromPCLPointCloud2(temp, *cloud);

    std::vector<point_t> centroids;
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

void processPointCloud(const cloud_t::Ptr &cloud, std::vector<point_t> &centroids) {
    // Passthrough filter 
    //cloud_t::Ptr pass_cloud(new cloud_t);
    //passthroughFilter(cloud, pass_cloud);

    // Downsample using Voxel Grid
    cloud_t::Ptr filtered_cloud(new cloud_t);
    downsample(cloud, filtered_cloud);

    // Calculate normals
    pcl::search::Search<point_t>::Ptr tree = boost::shared_ptr<pcl::search::Search<point_t> > (new pcl::search::KdTree<point_t>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<point_t, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (filtered_cloud);
    normal_estimator.setKSearch (40);
    normal_estimator.compute (*normals);

    // Region growing
    pcl::RegionGrowing<point_t, pcl::Normal> reg;

    //Maximum and minimum number of points to classify as a cluster
    // First check: to see if object is large (or takes up a large portion of the laser's view)
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);

    // Number of neighbors to search 
    reg.setNumberOfNeighbours(25);
    reg.setInputCloud(filtered_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(6.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(.1);

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

            for(std::vector<point_t>::iterator it = centroids.begin(); it != centroids.end(); ++it) {
                viewer.addSphere(*it, .1, "Sphere" + (it - centroids.begin()));    
            }
            while (!viewer.wasStopped())
            {
                viewer.spinOnce(100);
            }
        }
    }
}

void possibleDoors(std::vector<point_t> &new_doors, std::map<point_t, int, cmpPoints> &curr_doors, float threshold) {
    if (curr_doors.size() == 0) {
        for (int i = 0; i < new_doors.size(); i++) {
            curr_doors.insert(std::pair<point_t, int>(new_doors[i], 1));
        }
    } else {
        for (int i = 0; i < new_doors.size(); i++) {
        
            bool found_match = false;

            for (std::map<point_t, int>::iterator it = curr_doors.begin(); it != curr_doors.end(); ++it) {
                point_t door = it->first;
                float distance = dist(door, new_doors[i]);
                std::cout << "Distance: " << distance << std::endl;
        
                if (distance < threshold) {

                    // The counter in the map should be incremented
                    curr_doors[door]++;
                    found_match = true;
                    break;
                }
            }
            if (!found_match) {
                curr_doors.insert(std::pair<point_t, int>(new_doors[i], 1));
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

    std::map<point_t, int, cmpPoints> possible_doors;

    cloud_t::Ptr cloud(new cloud_t);

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
            if (pcl::io::loadPCDFile<point_t>(stream.str(), *cloud) == -1) {
                std::cout << "Reading failed for bag: " << i << std::endl;
                return (-1);
            }   
            std::cout << "Loading bag: " << i << ".pcd" << std::endl;
            std::vector<point_t> centroids;
            processPointCloud(cloud, centroids);
            num_tests_passed += (centroids.size() > 0) ? 1 : 0;     
        }
        std::cout << "PASSED " << num_tests_passed << " TESTS OUT OF 14" << std::endl;
    } else if (strcmp(argv[1], "test2") == 0) {
        // Read directory
        std::vector<boost::filesystem::path> paths;
        boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
        for ( boost::filesystem::directory_iterator itr("."); itr != end_itr; ++itr )
        {
            paths.push_back(itr->path());
        }
        
        std::sort(paths.begin(), paths.end());

        for (int i=0; i < paths.size(); i++) {
            if (pcl::io::loadPCDFile<point_t>(paths[i].string(), *cloud) == -1) {
                std::cout << "Reading failed for bag: " << paths[i] << std::endl;
                return (-1);
            }
            std::cout << "Loading bag: " << paths[i] << std::endl;
            std::vector<point_t> centroids;
            processPointCloud(cloud, centroids);
            
            // Accumulate possible doors
            possibleDoors(centroids, possible_doors, .6);
        }
    } else if (pcl::io::loadPCDFile<point_t>(argv[1], *cloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    } else {
        std::vector<point_t> centroids;
        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        //     transform.rotate(Eigen::AngleAxisf(1.3, Eigen::Vector3f::UnitZ()));
        //     cloud_t::Ptr rotated_cloud (new cloud_t());
        //     pcl::transformPointCloud(*cloud, *rotated_cloud, transform);
            processPointCloud(cloud, centroids);
    }    
    std::cout << "Number of possible doors: " << possible_doors.size() << std::endl;
    for (std::map<point_t, int>::iterator it = possible_doors.begin(); it != possible_doors.end(); it++){ 
        point_t p = it->first;
        std::cout << p.x << ", " << p.y << ", " << p.z << ": " << possible_doors[p] << std::endl;
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