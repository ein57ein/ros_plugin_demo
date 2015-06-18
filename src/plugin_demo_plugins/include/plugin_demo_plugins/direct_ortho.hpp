#ifndef ROS_PLUGIN_DEMO_DIRECT_ORTHO_H_
#define ROS_PLUGIN_DEMO_DIRECT_ORTHO_H_

#include <plugin_demo_interface/path_planner.hpp>

namespace plugin_demo_plugins_namespace
{
	class DirectOrtho : public plugin_demo_interface_namespace::PathPlanner
	{
		int getPath();

	public:
		int onInit(ros::NodeHandle roshandle);
			
		DirectOrtho() {}
		virtual ~DirectOrtho(){}
	};
};
#endif //ROS_PLUGIN_DEMO_DIRECT_ORTHO_H_
