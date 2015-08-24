/** \file direct.cpp
 * Source for the \link plugin_demo_plugins_namespace::Direct Direct \endlink class.
 * Tell ROS about this PlugIn for the \link plugin_demo_interface_namespace::PathPlanner Interface \endlink .
 * Author : Martin Seidel
 **/
#include <pluginlib/class_list_macros.h>
#include <plugin_demo_plugins/direct.hpp>

PLUGINLIB_EXPORT_CLASS(plugin_demo_plugins_namespace::Direct, plugin_demo_interface_namespace::PathPlanner)
