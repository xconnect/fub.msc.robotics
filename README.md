# Robotik
Freie Universität Berlin | VL Robotik WS 2015/16 

Man droht mir mit Punktabzug, wenn ich keinen Partner suche (HA!), deshalb pushe ich jetzt nach hier:
https://github.com/juauer/RO_1516
- Ju

For the ros stuff to work, it probably has to be initialised as described in the following link: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
delete the build-folder and run 'catkin_init_workspace' inside the src-folder, and then run 'catkin_make' in the home directory(ros).
After that source the setup.bash in ros/devel('source devel/setup.bash') to add the ros-directory to the ros-environment.

Using rviz: roslaunch [packagename] display.launch {model:=urdf/name.urdf}
Example: roslaunch arm2r display.launch

Starting roscore: roscore
Listing all running ros nodes: rosnode list
Starting turlesim: rosrun turtlesim turtlesim_node
With key-movement, in different terminal: rosrun turtlesim turtle_teleop_key
