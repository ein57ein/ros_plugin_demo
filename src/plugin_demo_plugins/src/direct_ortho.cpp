#include <pluginlib/class_list_macros.h>
#include <plugin_demo_plugins/direct_ortho.hpp>

PLUGINLIB_EXPORT_CLASS(plugin_demo_plugins_namespace::DirectOrtho, plugin_demo_interface_namespace::PathPlanner)

namespace plugin_demo_plugins_namespace
{
	int DirectOrtho::onInit(ros::NodeHandle roshandle)
	{
		ROS_INFO("[plugin direct_ortho] done init.");
	}

	int DirectOrtho::getPath()
	{
		current_path.poses.push_back(start);
		
		geometry_msgs::PoseStamped point;
		double yaw = ( getYawFromQuat(start.pose.orientation) + getYawFromQuat(target.pose.orientation) ) / 2;
		setPose2d(&point, start.pose.position.x, target.pose.position.y, yaw);
		current_path.poses.push_back(point);
		
		current_path.poses.push_back(target);
		ROS_INFO("[plugin direct_ortho] here comes the path");
	}
}
