#include <nodelet/nodelet.h>
#include <plugin_demo_plugins/direct.hpp>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace plugin_demo_plugins_namespace
{

    class DirectNodelet : public nodelet::Nodelet , public Direct
    {
		ros::Subscriber newStartNodeletSub;
		ros::Publisher setStartNodeletPub;

		void getStartNodeletCallback(const geometry_msgs::PoseWithCovarianceStamped event);
		
	public:
		void onInit();

		DirectNodelet() {}
		virtual ~DirectNodelet() {}
    };

}
