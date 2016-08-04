#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pub;
ros::Subscriber sub;
geometry_msgs::PointStamped prev;

void callback(const geometry_msgs::PointStamped &msg) 
{
	prev = msg;

	pub.publish(prev);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "point_repeater");
	ros::NodeHandle n;
		
	pub = n.advertise<geometry_msgs::PointStamped>("/door", 1000);
	
	sub = n.subscribe("/door2", 1000, &callback);
	
	while(ros::ok()) {
	
		if (ros::getGlobalCallbackQueue()->isEmpty()) {
		
			ROS_INFO("Publishing prev");

			ros::Duration(0.06).sleep();

			callback(prev);

		} else {

			ROS_INFO("Publishing new");

			ros::getGlobalCallbackQueue()->callOne(); 
		}
	
	}

	return 0;
}

