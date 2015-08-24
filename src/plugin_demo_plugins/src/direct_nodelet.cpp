#include <pluginlib/class_list_macros.h>
#include <plugin_demo_plugins/direct_nodelet.hpp>

PLUGINLIB_EXPORT_CLASS(plugin_demo_plugins_namespace::DirectNodelet, nodelet::Nodelet) 

namespace plugin_demo_plugins_namespace
{
    void DirectNodelet::onInit() 
    {
        NODELET_DEBUG("Initializing nodelet...");

        bool with_main = false;

        if (this->getMyArgv().size() > 0) {
			NODELET_INFO("test: %s", this->getMyArgv()[0].c_str());
			with_main = this->getMyArgv()[0].compare("with_main");
		}

		if (!with_main) {
			newStartNodeletSub = this->getNodeHandle().subscribe("get_start", 10, &DirectNodelet::getStartNodeletCallback, this);
			setStartNodeletPub = this->getNodeHandle().advertise<geometry_msgs::Pose2D>("set_start", 10);
		}
	
        initialize( this->getNodeHandle() );
    }

    void DirectNodelet::getStartNodeletCallback(const geometry_msgs::PoseWithCovarianceStamped event)
	{
		geometry_msgs::Pose2DPtr msg(new geometry_msgs::Pose2D);
		msg->x = event.pose.pose.position.x;
		msg->y = event.pose.pose.position.y;
		msg->theta = getYawFromQuat(event.pose.pose.orientation);
		setStartNodeletPub.publish(msg);
	}
}
