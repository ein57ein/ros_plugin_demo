#ifndef ROS_PLUGIN_DEMO_DIRECT_H_
#define ROS_PLUGIN_DEMO_DIRECT_H_

#include <plugin_demo_interface/path_planner.hpp>

namespace plugin_demo_plugins_namespace
{
	class Direct : public plugin_demo_interface_namespace::PathPlanner
	{
		int getPath();

	public:
		int onInit(ros::NodeHandle roshandle);
			
		Direct() {}
		virtual ~Direct(){}
	};
};
#endif //ROS_PLUGIN_DEMO_DIRECT_H_
