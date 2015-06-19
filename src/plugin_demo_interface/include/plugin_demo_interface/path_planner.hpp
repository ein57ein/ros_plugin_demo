#ifndef ROS_PLUGIN_DEMO_PATHPLANNER_H_
#define ROS_PLUGIN_DEMO_PATHPLANNER_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_broadcaster.h>

namespace plugin_demo_interface_namespace
{
	class PathPlanner
	{
		ros::Subscriber getTargetSub;
		ros::Subscriber getStartSub;
		ros::Publisher pathPub;
		ros::Timer sendPath;
		tf::TransformBroadcaster tf_br;
		
		void getTargetCallback(const geometry_msgs::PoseStamped event);
		void sendPathCallback(const ros::TimerEvent& event);
		void getStarttCallback(const geometry_msgs::Pose2D event);
		
	protected:
		bool init;

		geometry_msgs::PoseStamped start;
		geometry_msgs::PoseStamped target;
		nav_msgs::Path current_path;
		
		PathPlanner();
		virtual int onInit(ros::NodeHandle roshandle);
		virtual int getPath() = 0;
		int setPose2d(geometry_msgs::PoseStamped *pose3d, double x, double y, double theta);
			
	public:
		int initialize(ros::NodeHandle roshandle);
		double getYawFromQuat(geometry_msgs::Quaternion quat);

		int setPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target);
		int getPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target);
		
		virtual ~PathPlanner() {}
	};
};
#endif //ROS_PLUGIN_DEMO_PATHPLANNER_H_
