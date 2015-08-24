/** \file direct.hpp
 * Contain the \link plugin_demo_plugins_namespace::Direct Direct \endlink class.
 * Author : Martin Seidel
 **/
#ifndef ROS_PLUGIN_DEMO_DIRECT_H_
#define ROS_PLUGIN_DEMO_DIRECT_H_

#include <plugin_demo_interface/path_planner.hpp>

/**Contain example plugIns for the \link
 * plugin_demo_interface_namespace::PathPlanner Interface \endlink .
 **/
namespace plugin_demo_plugins_namespace
{
	/**This class create a direct link between the start and target pose.
	 * The resulting path only contain these two points.
	 **/
	class Direct : public plugin_demo_interface_namespace::PathPlanner
	{
		/** "calculate" the path.
		 * @return error code
		 **/
		int getPath() {
			current_path.poses.push_back(start);
			current_path.poses.push_back(target);
			ROS_INFO("[plugin direct] here comes the path");
		}

		/** writes only a info-msg.
		 * @param roshandle a valid ROS NodeHandle
		 * @return error code
		 **/
		int onInit(ros::NodeHandle roshandle) {
			ROS_INFO("[plugin direct] done init.");
		}
		
	public:
		Direct() {}	/**< Intentionally left empty **/
		virtual ~Direct(){}	/**< Intentionally left empty **/
	};
};
#endif //ROS_PLUGIN_DEMO_DIRECT_H_
