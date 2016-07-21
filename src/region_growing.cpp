#include "region_growing.h"

#define LOAD_FILE

inline float dist(point_t a, point_t b) 
{
    vec3 a2(a.x, a.y, a.z);
    vec3 b2(b.x, b.y, b.z);
    return (a2 - b2).norm();
}

// Comparison function to sort doorPoints
bool cmpFreq(const doorPoint &a, const doorPoint &b) 
{
    return (a.freq > b.freq);
}

void findDoorCentroids(const cloud_t::Ptr &cloud, const std::vector<pcl::PointIndices> &indices, std::vector<point_t> &centroids) 
{
    // X-coord width in meters
    const float min_width = .8;
    const float max_width = .96;

    // Loop through clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it) {

        // Create cluster from indices
        cloud_t::Ptr cluster(new cloud_t(*cloud, (*it).indices));        

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

void downsample(const cloud_t::Ptr &cloud, cloud_t::Ptr &filtered_cloud) 
{
    pcl::VoxelGrid<point_t> filter;
    filter.setInputCloud(cloud);

    // Filter size may need to be adjusted  
    filter.setLeafSize(.017, .017, .017);
    //std::cout << "Original size: " << cloud->points.size() << std::endl;
    filter.filter(*filtered_cloud);
    std::cout << "Reduced size: " << filtered_cloud->points.size() << std::endl;
}

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
    pcl::PCLPointCloud2 temp;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, temp);
    cloud_t::Ptr cloud(new cloud_t);
    pcl::fromPCLPointCloud2(temp, *cloud);

    std::vector<point_t> centroids;

    processPointCloud(cloud, centroids);

    if (centroids.size() > 0) {
        publishDoor(centroids[0]);
    }
    /*possibleDoors(centroids, .45);

    std::sort(curr_doors.begin(), curr_doors.end(), cmpFreq);

    //for (int i = 0; i < std::min(3, (int)curr_doors.size()); ++i) {
    int i = 0;
	while (i < curr_doors.size()) {
        if (curr_doors[i].center.z < 0) {
            i++;
        } else {
            std::cout << "Publishing " << curr_doors[i].center.x << 
            ", " << curr_doors[i].center.y << ", " << 
            curr_doors[i].center.z << ". With freq " << curr_doors[i].freq << std::endl;
    	   
        	publishDoor(curr_doors[i].center);
            break;
        }
    }*/
}

void publishDoor(point_t centroid) 
{

    geometry_msgs::PointStamped ps;
    geometry_msgs::Point point;
    
    point.x = centroid.x;
    point.y = centroid.y;
    point.z = centroid.z;
    ps.point = point;

    std_msgs::Header header;
    header.seq = header_seq++;
    header.frame_id = "/world";
    header.stamp = ros::Time::now();
    ps.header = header;
    
    pub.publish(ps);
}

void processPointCloud(const cloud_t::Ptr &cloud, std::vector<point_t> &centroids) 
{

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
    reg.setMinClusterSize(120);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);

    // Number of neighbors to search 
    reg.setNumberOfNeighbours(20);
    reg.setInputCloud(filtered_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(6.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(.2);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // Update colored cloud
    colored_cloud = reg.getColoredCloud();

    // Use clusters to find door(s)
    findDoorCentroids(filtered_cloud, clusters, centroids);
}

void possibleDoors(std::vector<point_t> &new_doors, float threshold) 
{
    std::cout << "Found " << new_doors.size() << " new doors." << std::endl;
    if (curr_doors.size() == 0) {
        for (int i = 0; i < new_doors.size(); i++) {
            doorPoint p;
            p.freq = 1;
            p.center = new_doors[i];
            curr_doors.push_back(p);
        }
    } else {
        for (int i = 0; i < new_doors.size(); i++) {
        
            bool found_match = false;

            for (std::vector<doorPoint>::iterator it = curr_doors.begin(); it != curr_doors.end(); ++it) {
                point_t door = it->center;
                float distance = dist(door, new_doors[i]);
                //std::cout << "Distance: " << distance << std::endl;
        
                if (distance < threshold) {
                    // The counter in the map should be incremented
                    it->freq++;
                    found_match = true;
                    break;
                }
            }
            if (!found_match) {
                doorPoint p;
                p.freq = 1;
                p.center = new_doors[i];
                curr_doors.push_back(p);
            }
        }
    }
}

void showPointCLoud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<pcl::PointXYZ> &centroids) {
    if (strcmp(argv[2], "-c") == 0) {
        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        viewer.showCloud(cloud);
        while (!viewer.wasStopped())
        {
        }
    } else if (strcmp(argv[2], "-d") == 0) {
        pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
        viewer.setBackgroundColor(0,0,0);
        viewer.initCameraParameters();
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");

        for(std::vector<point_t>::iterator it = centroids.begin(); it != centroids.end(); ++it) {
            viewer.addSphere(*it, .1, "Sphere" + (it - centroids.begin()));    
        }
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_pcl");
    ros::NodeHandle n;

    // Publisher
    pub = n.advertise<geometry_msgs::PointStamped>("door", 1);

    ::argc = argc;
    ::argv = argv;

    cloud_t::Ptr cloud(new cloud_t);

    if (argc < 2) {
        std::cout << "Now subscribing to ROS topics" << std::endl;
        // Run in realtime through ROS
        ros::Subscriber sub = n.subscribe("cloud", 1, cloudCB);

        ros::spin();

        return (0);
    } else if (strcmp(argv[1], "test") == 0) {
        // Read all files in directory

        std::vector<boost::filesystem::path> paths;
        boost::filesystem::directory_iterator end_itr; 

        for ( boost::filesystem::directory_iterator itr("."); itr != end_itr; ++itr )
        {
            paths.push_back(itr->path());
        }
        
        std::sort(paths.begin(), paths.end());

		ros::Rate r(2.);

        for (int i=0; i < paths.size(); i++) {
            if (pcl::io::loadPCDFile<point_t>(paths[i].string(), *cloud) == -1) {
                std::cout << "Reading failed for bag: " << paths[i] << std::endl;
                return (-1);
            }

            std::cout << "Loading bag: " << paths[i] << std::endl;
            std::vector<point_t> centroids;
            processPointCloud(cloud, centroids);
            
            if (centroids.size() > 0) {
                if (centroids[0].z > 0) {
                    publishDoor(centroids[0]);
                }
            }
            
            // Accumulate possible doors
            /*possibleDoors(centroids, .35);

	        std::sort(curr_doors.begin(), curr_doors.end(), cmpFreq);
	        if (curr_doors.size() > 0) {
	        	
	        	int i=0;
	        	
	        	// Very preliminary filter
	        	while(i < curr_doors.size()) {
	        		if (curr_doors[i].center.z < 0) {
	        			i++;
	        		} else {
                        std::cout << "Publishing " << curr_doors[i].center.x << 
                        ", " << curr_doors[i].center.y << ", " << 
                        curr_doors[i].center.z << ". With freq " << curr_doors[i].freq << std::endl;
                        publishDoor(curr_doors[i].center);
                        break;
	        		}
	        	}
	        }*/

	        r.sleep();
        }

        std::cout << "Number of possible doors: " << curr_doors.size() << std::endl;
		std::cout << "Printing doors with Y > 0" << std::endl;
        
        for (std::vector<doorPoint>::iterator it = curr_doors.begin(); it != curr_doors.end(); it++){ 
            point_t p = it->center;
            if (p.y > 0) {
            	std::cout << p.x << ", " << p.y << ", " << p.z << ": " << it->freq << std::endl;
            }
        }

    } else {
        if (pcl::io::loadPCDFile<point_t>(argv[1], *cloud) == -1) {
            std::cout << "Cloud reading failed." << std::endl;
            return (-1);
        }

        std::vector<point_t> centroids;
        processPointCloud(cloud, centroids);
        
        // Show pointcloud
        if (argc > 2) {
            showPointCLoud(colored_cloud, centroids);
        }
    }    
    return (0);
}