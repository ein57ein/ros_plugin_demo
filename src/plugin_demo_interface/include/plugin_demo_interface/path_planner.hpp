/** \file path_planner.hpp
 * Contain the \link plugin_demo_interface_namespace::PathPlanner Interface \endlink class.
 * Author : Martin Seidel
 **/
#ifndef ROS_PLUGIN_DEMO_PATHPLANNER_H_
#define ROS_PLUGIN_DEMO_PATHPLANNER_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_broadcaster.h>

/**Contain the \link
 * plugin_demo_interface_namespace::PathPlanner Interface \endlink .
 **/
namespace plugin_demo_interface_namespace
{
	/**Contain the \link plugin_demo_interface_namespace::PathPlanner
	 * Interface \endlink . Serve some mutual functions for all
	 * inherited plugIns.
	 **/
	class PathPlanner
	{
		ros::Subscriber getTargetSub;	/**< listen for a new target pose **/
		ros::Subscriber getStartSub;	/**< listen for a new start pose **/
		ros::Publisher pathPub;	/**< send the path calculated by the plugIn **/
		ros::Timer sendPath;	/**< periodic calling of the path publisher **/
		tf::TransformBroadcaster tf_br;	/**< send tf-transforms **/

		/** receives a new #target pose and call getPath().
		 * @param event a pose message from rViz
		 **/
		void getTargetCallback(const geometry_msgs::PoseStamped event);
		
		/** publish #current_path and the poses #start and #target as a tf-frame
		 * @param event some time values
		 **/
		void sendPathCallback(const ros::TimerEvent& event);
		
		/** receives a new #start pose and call getPath().
		 * @param event a pose message from the main part of this demo
		 **/
		void getStarttCallback(const geometry_msgs::Pose2D event);
		
	protected:
		bool init;	/**< true if initialized. **/

		geometry_msgs::PoseStamped start;	/**< starting pose of the path **/
		geometry_msgs::PoseStamped target;	/**< last pose of the path **/
		nav_msgs::Path current_path;	/**< contain the path **/

		/** initialize #init
		 **/
		PathPlanner();
		
		/** In this function the plugIn can do some initialisation
		 * depending on a ROS NodeHandle.
		 * @param roshandle a valid ROS NodeHandle
		 * @return error code
		 **/
		virtual int onInit(ros::NodeHandle roshandle);
		
		/** calculate #current_path from #start and #target inside a plugIn.
		 * @return error code
		 **/
		virtual int getPath() = 0;

		/** put a given 2D-Pose into 3D-Stamped-Pose.
		 * @param pose3d pointer to the target variable
		 * @param x the x-value of the 2D-Pose
		 * @param y the y-value of the 2D-Pose
		 * @param theta the theta-value of the 2D-Pose
		 **/
		void setPose2d(geometry_msgs::PoseStamped *pose3d, double x, double y, double theta);
			
	public:
		int initialize(ros::NodeHandle roshandle);
		double getYawFromQuat(geometry_msgs::Quaternion quat);

		int setPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target);
		int getPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target);
		
		virtual ~PathPlanner() {}	/**< Intentionally left empty **/
	};
};
#endif //ROS_PLUGIN_DEMO_PATHPLANNER_H_
