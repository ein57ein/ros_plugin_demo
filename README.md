#Demonstration of the ROS PlugInLib
This project is devided into three ROS-packages:

* [plugin_demo_interface](https://github.com/ein57ein/ros_plugin_demo/tree/master/src/plugin_demo_interface "PlugIn-Interface"): The PlugIn-Interface for this example
* [plugin_demo_plugins](https://github.com/ein57ein/ros_plugin_demo/tree/master/src/plugin_demo_plugins "Example PlugIns"): Some PlugIns for this Interface
* [plugin_demo_main](https://github.com/ein57ein/ros_plugin_demo/tree/master/src/plugin_demo_main "PlugIn-User"): An executable, which includes the Interface and load one of the PlugIns dynamically

The [visual_aids](https://github.com/ein57ein/ros_plugin_demo/tree/master/src/visual_aids "visual_aids") package include a tool which helps to show the orientation of points from a path using a PoseArray.

There is also a more comprehensive [documentation](http://ein57ein.github.io/ros_plugin_demo/ "doxyGenDoc") available.

##The basic communication between the _plugin_demo_ packages:

![Sometimes you can see an image of the basic communication structure](https://github.com/ein57ein/ros_plugin_demo/blob/master/communication.png "basic communication structure")

* functions are marked with brackets __()__ at the end
* a function which have a "__Callback__" in his name is assigned to a ROS::Subscriber
* __setStartPub__ is a ROS::Publisher
* __rViz__ is the ROS tool (is used for setting poses)
* a plugIn could also contain ROS::Subscribers or ROS::Publishers
 
## Usage

1. compile the packages from this repository:
    * clone this repository, add it to your .bashrc and call catkin_make inside the root folder of this repository  
    ___XOR___  
    * download only the packages, put them into an existing catkin workspace and compile them in this workspace.
2. run a ROS-Master: `roscore`
3. run rViZ: `rosrun rviz rviz`
    * load the [rViZ config](https://github.com/ein57ein/ros_plugin_demo/blob/master/ros_plugin_demo.rviz "config") included in this repository
4. start the plugin_demo: `roslaunch plugin_demo_main start_plugin_demo.launch`
5. start dynamic reconfigure: `rosrun rqt_reconfigure rqt_reconfigure`
6. have fun.
    * You can set in rViz
        * the __start tf-frame__ with "2D Pose Estimate" (/get_start)
        * the __target tf-frame__ with "2D Nav Goal" (/set_target)
    * You can control some parameters with dynamic reconfigure ("plugin_demo")
        * if you check __use_shared_pointer__ new poses for the __start tf-frame__ will be transmitted with shared pointers between the [main executable](https://github.com/ein57ein/ros_plugin_demo/blob/master/src/plugin_demo_main/src/main.cpp#L18#L25 "Line 18 to 25") and the [Interface](https://github.com/ein57ein/ros_plugin_demo/blob/master/src/plugin_demo_interface/src/path_planner.cpp#L86 "Line 86")
        * the values of the parameters __delta_x__, __delta_y__ and __delta_theta__ are added to the values of a new start pose in the [main executable](https://github.com/ein57ein/ros_plugin_demo/blob/master/src/plugin_demo_main/src/main.cpp#L14#L36 "Line 14 to 36")
        * the last parameter change the __plugin__ which calculate the path. The values in the Dropdown menu are hard coded in the [plugin_demo.cfg](https://github.com/ein57ein/ros_plugin_demo/blob/master/src/plugin_demo_main/cfg/plugin_demo.cfg "plugin_demo.cfg"). So if you build a new plugIn add it also there.

