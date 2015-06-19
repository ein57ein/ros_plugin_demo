#include "plugin_demo_interface/path_planner.hpp"

namespace plugin_demo_interface_namespace
{
	PathPlanner::PathPlanner()
	{
		init = false;
	}
	
	int PathPlanner::initialize(ros::NodeHandle roshandle)
	{
		if (init) {
			ROS_WARN("[interface] plugin was already initialized. Nothing will be done.");
		} else {
			getTargetSub = roshandle.subscribe("set_target", 10, &PathPlanner::getTargetCallback, this);
			getStartSub = roshandle.subscribe("set_start", 10, &PathPlanner::getStarttCallback, this);
			pathPub = roshandle.advertise<nav_msgs::Path>("path", 10);
			sendPath = roshandle.createTimer(ros::Rate(1.0), &PathPlanner::sendPathCallback, this);

			setPose2d(&start, 0, 0, 0);
			target = start;
			target.header.stamp = ros::Time::now();

			current_path.header.stamp = ros::Time::now();
			current_path.header.frame_id = "world";
			current_path.poses.push_back(start);
			current_path.poses.push_back(target);
			
			onInit(roshandle);
			init = true;
		}
	}

	int PathPlanner::onInit(ros::NodeHandle roshandle)
	{
		ROS_INFO("[interface] plugIn has no onInit function");
	}

	int PathPlanner::setPose2d(geometry_msgs::PoseStamped *pose3d, double x, double y, double theta)
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

		current_path.poses.clear();
		getPath();

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

	void PathPlanner::getStarttCallback(const geometry_msgs::Pose2D event)
	{
		setPose2d(&start, event.x, event.y, event.theta);
		current_path.poses.clear();
		getPath();
	}

	void PathPlanner::getTargetCallback(const geometry_msgs::PoseStamped event)
	{
		target = event;
		current_path.poses.clear();
		getPath();
	}

	void PathPlanner::sendPathCallback(const ros::TimerEvent& event)
	{
		tf::Transform transform;
		
		transform.setOrigin( tf::Vector3(start.pose.position.x, start.pose.position.y, start.pose.position.z) );
		tf::Quaternion q(start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w);
		transform.setRotation(q);
		tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "start"));

		transform.setOrigin( tf::Vector3(target.pose.position.x, target.pose.position.y, target.pose.position.z) );
		tf::Quaternion q2(target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w);
		transform.setRotation(q2);
		tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));
		
		current_path.header.stamp = ros::Time::now();
		pathPub.publish(current_path);		
	}
	
}
