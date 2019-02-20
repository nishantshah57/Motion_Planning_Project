// This code used MoveIt library and generates a plan
// for the PR2's left arm to move to a random target position

// MoveIt Header file
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_target_manipulation_node",
  ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  // Always start spinner if using moveit
  spinner.start();
  // this connects to a running instance of the move_group node
  // Here the Planning group is "left_arm"
  move_group_interface::MoveGroup left_arm_group("left_arm");
  // specify that our target will be a random one
  left_arm_group.setRandomTarget();
  // plan the motion and then move the group to the sampled target
  left_arm_group.move();
  ros::waitForShutdown();
}
