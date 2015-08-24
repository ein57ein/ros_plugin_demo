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
		ros::Timer sendPathTimer;	/**< periodic calling of the path publisher **/

		tf::Transform start_tf;	/**< the start tf-frame **/
		tf::Transform target_tf;	/**< the target tf-frame **/
		tf::TransformBroadcaster tf_br;	/**< send tf-transforms **/

		/**receives a new #target pose and call getPath().
		 * @param event a pose message from rViz
		 **/
		void getTargetCallback(const geometry_msgs::PoseStamped event);
		
		/**call sendPath()
		 * @param event some time values
		 **/
		void sendPathCallback(const ros::TimerEvent& event);
		
		/**receives a new #start pose and call getPath().
		 * @param event a pose message from the main part of this demo
		 **/
		void getStartCallback(const geometry_msgs::Pose2D event);

		/**publish #current_path and the #start_tf and #target_tf frames
		 **/
		void sendPath();
		
	protected:
		bool init;	/**< true if initialized. **/

		geometry_msgs::PoseStamped start;	/**< starting pose of the path **/
		geometry_msgs::PoseStamped target;	/**< last pose of the path **/
		nav_msgs::Path current_path;	/**< contain the path **/

		/** initialize #init
		 **/
		PathPlanner();
		
		/**In this function the plugIn can do some initialisation
		 * depending on a ROS NodeHandle.
		 * @param roshandle a valid ROS NodeHandle
		 * @return error code
		 **/
		virtual int onInit(ros::NodeHandle roshandle);
		
		/**calculate #current_path from #start and #target inside a plugIn.
		 * @return error code
		 **/
		virtual int getPath() = 0;

		/**convert a given 2D-Pose into 3D-Stamped-Pose.
		 * @param pose3d pointer to the target variable
		 * @param x the x-value of the 2D-Pose
		 * @param y the y-value of the 2D-Pose
		 * @param theta the theta-value of the 2D-Pose
		 **/
		void setPose2d(geometry_msgs::PoseStamped *pose3d, double x, double y, double theta);
			
	public:
		/**Initalize the global variables.
		 * @param roshandle a valid ROS NodeHandle
		 **/
		void initialize(ros::NodeHandle roshandle);

		/**convert a quaternion into yaw
		 * @param quat a Quaternion
		 * @return the yaw angle of the quaternion
		 **/
		static double getYawFromQuat(geometry_msgs::Quaternion quat);

		/**build a tf-frame for a given pose
		 * @param pose the pose for the tf-frame
		 * @param tf_tf the resulting tf-frame
		 **/
		void tfFrameFromPoseStamped(geometry_msgs::PoseStamped *pose, tf::Transform *tf_tf);

		/**replace the current #start and #target poses and call getPath()
		 * @param start new start pose
		 * @param target new target pose
		 * @return error code
		 **/
		int setPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target);
		
		/**returns the current #start and #target poses
		 * @param start current start pose
		 * @param target current target pose
		 * @return error code
		 **/
		int getPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target);
		
		virtual ~PathPlanner() {}	/**< Intentionally left empty **/
	};
};
#endif //ROS_PLUGIN_DEMO_PATHPLANNER_H_
