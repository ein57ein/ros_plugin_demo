#include <nodelet/nodelet.h>
#include <plugin_demo_plugins/direct_ortho.hpp>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace plugin_demo_plugins_namespace
{

    class DirectOrthoNodelet : public nodelet::Nodelet , public DirectOrtho
    {
		ros::Subscriber newStartNodeletSub;
		ros::Publisher setStartNodeletPub;

		void getStartNodeletCallback(const geometry_msgs::PoseWithCovarianceStamped event);
		
	public:
		void onInit();

		DirectOrthoNodelet() {}
		virtual ~DirectOrthoNodelet() {}
    };

}
