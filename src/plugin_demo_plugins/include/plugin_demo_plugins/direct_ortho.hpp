/** \file direct_ortho.hpp
 * Contain the \link plugin_demo_plugins_namespace::DirectOrtho DirectOrtho \endlink class.
 * Author : Martin Seidel
 **/
#ifndef ROS_PLUGIN_DEMO_DIRECT_ORTHO_H_
#define ROS_PLUGIN_DEMO_DIRECT_ORTHO_H_

#include <plugin_demo_interface/path_planner.hpp>

namespace plugin_demo_plugins_namespace
{
	/**This class create a path parallel to the axies of the coordinate
	 * system between the start and target pose. The resulting path only
	 * contain three points. The one in the middle takes the x-value
	 * from the start pose and the y-value from the target pose. Theta
	 * is the average of the thetas from the two other poses.
	 **/
	class DirectOrtho : public plugin_demo_interface_namespace::PathPlanner
	{
		/** calculate the third point of the resulting path.
		 * @return error code
		 **/
		int getPath();

		/** writes only a info-msg.
		 * @param roshandle a valid ROS NodeHandle
		 * @return error code
		 **/
		int onInit(ros::NodeHandle roshandle);
		 
	public:			
		DirectOrtho() {}	/**< Intentionally left empty **/
		virtual ~DirectOrtho(){}	/**< Intentionally left empty **/
	};
};
#endif //ROS_PLUGIN_DEMO_DIRECT_ORTHO_H_
