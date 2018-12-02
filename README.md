# Motion_Planning_Project
Manipulation and Grasping using PR2 Robot
We are using MoveIt! framework to account for inverse kinmeatics, collision checking and generate smooth paths.
Make sure ROS has been installed properly with all the packages.
It needs PR2 description package pre-installed.
Source the ROS path properly before launching the model and catkin_make the package.

To launch the system:
$ roslaunch world_launch pr2_gazebo_moveit.launch

To run a node from manipulation_and_grasping package, enter the line below in a different terminal:
$ rosrun manipulation_and_grasping move_arm
or 
$ rosrun manipulation_and_grasping random_target_manipulation
