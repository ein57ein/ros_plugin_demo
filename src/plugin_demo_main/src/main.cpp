#include "plugin_demo_interface/path_planner.hpp"
#include <pluginlib/class_loader.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

boost::shared_ptr<plugin_demo_interface_namespace::PathPlanner> plugin_node;
ros::Publisher setStartPub;

void getStartCallback(const geometry_msgs::PoseWithCovarianceStamped event)
{
	geometry_msgs::Pose2DPtr msg(new geometry_msgs::Pose2D);
	msg->x = event.pose.pose.position.x;
	msg->y = event.pose.pose.position.y;
	msg->theta = plugin_node->getYawFromQuat(event.pose.pose.orientation);
	setStartPub.publish(msg);
	//msg->x += 0.5; //shared pointer demo
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plugin_demo_path_planner");
	ros::NodeHandle roshandle;

	ros::Subscriber newStartSub;
	newStartSub = roshandle.subscribe("get_start", 10, &getStartCallback);
	setStartPub = roshandle.advertise<geometry_msgs::Pose2D>("set_start", 10);

	//the plugin_name should include the namespace
	std::string plugin_name;
	ros::param::param<std::string>("~path_planner_plugin", plugin_name, "plugin_demo_plugins_namespace::Direct");
	
	pluginlib::ClassLoader<plugin_demo_interface_namespace::PathPlanner> plug_in_loader("plugin_demo_interface", "plugin_demo_interface_namespace::PathPlanner");

	ROS_INFO("[main] starting plugin \"%s\"", plugin_name.c_str());
	try
	{
		plugin_node = plug_in_loader.createInstance(plugin_name.c_str());
		plugin_node->initialize(roshandle);		
	} catch(pluginlib::PluginlibException& ex) {
		ROS_FATAL("[main] failed to load. Error: \"%s\"\nGoodbye.", ex.what());
		return 1;
	}
	ROS_INFO("[main] load module");

	plugin_node->initialize(roshandle);
	ros::spin();
	
	return 0;
}
