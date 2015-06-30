/** \file path_planner.cpp
 * Source for the \link plugin_demo_interface_namespace::PathPlanner Interface \endlink class.
 * Author : Martin Seidel
 **/
#include "plugin_demo_interface/path_planner.hpp"

namespace plugin_demo_interface_namespace
{
	PathPlanner::PathPlanner()
	{
		init = false;
	}
	
	void PathPlanner::initialize(ros::NodeHandle roshandle)
	{
		if (init) {
			ROS_WARN("[interface] plugin was already initialized. Nothing will be done.");
		} else {
			getTargetSub = roshandle.subscribe("set_target", 10, &PathPlanner::getTargetCallback, this);
			getStartSub = roshandle.subscribe("set_start", 10, &PathPlanner::getStarttCallback, this);
			pathPub = roshandle.advertise<nav_msgs::Path>("path", 10);
			sendPathTimer = roshandle.createTimer(ros::Duration(2.0), &PathPlanner::sendPathCallback, this);

			setPose2d(&start, 0, 0, 0);
			target = start;
			target.header.stamp = ros::Time::now();

			tfFrameFromPoseStamped(&start, &start_tf);
			tfFrameFromPoseStamped(&target, &target_tf);

			current_path.header.stamp = ros::Time::now();
			current_path.header.frame_id = "world";
			current_path.poses.push_back(start);
			current_path.poses.push_back(target);
			
			onInit(roshandle);
			init = true;

			sendPath();
		}
	}

	int PathPlanner::onInit(ros::NodeHandle roshandle)
	{
		ROS_INFO("[interface] plugIn has no onInit function");
	}

	void PathPlanner::setPose2d(geometry_msgs::PoseStamped *pose3d, double x, double y, double theta)
	{
		pose3d->header.stamp = ros::Time::now();
		pose3d->pose.position.x = x;
		pose3d->pose.position.y = y;
		pose3d->pose.position.z = 0;
		pose3d->pose.orientation.x = 0;
		pose3d->pose.orientation.y = 0;
		pose3d->pose.orientation.z = sin(theta * 0.5);			
		pose3d->pose.orientation.w = cos(theta * 0.5);
	}

	int PathPlanner::setPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target)
	{
		setPose2d( &(this->start), start->x, start->y, start->theta);
		setPose2d( &(this->target), target->x, target->y, target->theta);

		tfFrameFromPoseStamped(&(this->start), &start_tf);
		tfFrameFromPoseStamped(&(this->target), &target_tf);

		current_path.poses.clear();
		getPath();

		sendPath();

		return 0;
	}

	int PathPlanner::getPoints(geometry_msgs::Pose2D *start, geometry_msgs::Pose2D *target)
	{
		start->x = this->start.pose.position.x;
		start->y = this->start.pose.position.y;
		start->theta = getYawFromQuat(this->start.pose.orientation);

		target->x = this->target.pose.position.x;
		target->y = this->target.pose.position.y;
		target->theta = getYawFromQuat(this->target.pose.orientation);

		return 0;
	}

	double PathPlanner::getYawFromQuat(geometry_msgs::Quaternion quat)
	{
		//geometry_msgs::Quaternion quat = ;
		tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		return yaw;
	}

	void PathPlanner::tfFrameFromPoseStamped(geometry_msgs::PoseStamped *pose, tf::Transform *tf_tf)
	{
		tf_tf->setOrigin( tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z) );
		tf::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
		tf_tf->setRotation(q);
	}

	void PathPlanner::getStarttCallback(const geometry_msgs::Pose2D event)
	{
		setPose2d(&start, event.x, event.y, event.theta);

		tfFrameFromPoseStamped(&start, &start_tf);
		
		current_path.poses.clear();
		getPath();

		sendPath();
	}

	void PathPlanner::getTargetCallback(const geometry_msgs::PoseStamped event)
	{
		target = event;

		tfFrameFromPoseStamped(&target, &target_tf);
		
		current_path.poses.clear();
		getPath();

		sendPath();
	}

	void PathPlanner::sendPathCallback(const ros::TimerEvent& event)
	{
		sendPath();
	}

	void PathPlanner::sendPath()
	{
		tf_br.sendTransform(tf::StampedTransform(start_tf, ros::Time::now(), "world", "start"));
		tf_br.sendTransform(tf::StampedTransform(target_tf, ros::Time::now(), "world", "target"));
		
		current_path.header.stamp = ros::Time::now();
		pathPub.publish(current_path);
	}	
}
