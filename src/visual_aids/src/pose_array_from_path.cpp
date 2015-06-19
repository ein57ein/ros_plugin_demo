#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"

ros::Publisher setPoseArrayPub;

void getPathCallback(const nav_msgs::Path event)
{
	geometry_msgs::PoseArray msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = event.header.frame_id;

	//for(const std::vector<geometry_msgs::PoseStamped>::iterator it=event.poses.begin(); it != event.poses.end(); it++) {
	for(int i=0; i < event.poses.size(); i++) {
		msg.poses.push_back(event.poses.at(i).pose);
	}

	setPoseArrayPub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_array_from_path");
	ros::NodeHandle roshandle;

	ros::Subscriber getPathSub = roshandle.subscribe("/path", 10, &getPathCallback);
	setPoseArrayPub = roshandle.advertise<geometry_msgs::PoseArray>("pose_array", 10);
	
	ros::spin();
		
	return 0;
}
