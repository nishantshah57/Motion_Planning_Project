// Includes ros libraries and packages
#include <ros/ros.h>

// MoveIt! interfaces and geometric shapes
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>

// For loading the robot description parameter from the server
static const std::string ROBOT_DESCRIPTION="robot_description";

// Open the gripper
// Input: Trajectory waypoints
// Out: None
void openGripper(trajectory_msgs::JointTrajectory& posture){
  posture.joint_names.resize(6);
  posture.joint_names[0] = "r_gripper_joint";
  posture.joint_names[1] = "r_gripper_motor_screw_joint";
  posture.joint_names[2] = "r_gripper_l_finger_joint";
  posture.joint_names[3] = "r_gripper_r_finger_joint";
  posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 1;
  posture.points[0].positions[1] = 1.0;
  posture.points[0].positions[2] = 0.477;
  posture.points[0].positions[3] = 0.477;
  posture.points[0].positions[4] = 0.477;
  posture.points[0].positions[5] = 0.477;
}

// Closing the gripper
// Input: Trajectory waypoints
// Out: None
void closedGripper(trajectory_msgs::JointTrajectory& posture){
  posture.joint_names.resize(6);
  posture.joint_names[0] = "r_gripper_joint";
  posture.joint_names[1] = "r_gripper_motor_screw_joint";
  posture.joint_names[2] = "r_gripper_l_finger_joint";
  posture.joint_names[3] = "r_gripper_r_finger_joint";
  posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;
  posture.points[0].positions[2] = 0.002;
  posture.points[0].positions[3] = 0.002;
  posture.points[0].positions[4] = 0.002;
  posture.points[0].positions[5] = 0.002;
}

// Using the Move group interface to generate plan for right arm
void pick(moveit::planning_interface::MoveGroup &right_arm_group)
{
  // grasps will have all the waypoints
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped pick_object;
  pick_object.header.frame_id = "base_footprint";
  pick_object.pose.position.x = 0.34;
  pick_object.pose.position.y = -0.7;
  pick_object.pose.position.z = 0.5;
  pick_object.pose.orientation.x = 0;
  pick_object.pose.orientation.y = 0;
  pick_object.pose.orientation.z = 0;
  pick_object.pose.orientation.w = 1;
  moveit_msgs::Grasp grasp_pick;
  grasp_pick.grasp_pose = pick_object;

  grasp_pick.pre_grasp_approach.direction.vector.x = 1.0;
  grasp_pick.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  grasp_pick.pre_grasp_approach.min_distance = 0.2;
  grasp_pick.pre_grasp_approach.desired_distance = 0.4;

  grasp_pick.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasp_pick.post_grasp_retreat.direction.vector.z = 1.0;
  grasp_pick.post_grasp_retreat.min_distance = 0.1;
  grasp_pick.post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasp_pick.pre_grasp_posture);
  // right_arm_group.attachObject("object");
  closedGripper(grasp_pick.grasp_posture);

  grasps.push_back(grasp_pick);
  right_arm_group.setSupportSurfaceName("table");
  right_arm_group.pick("object", grasps);

}

// Using the Move group interface to generate plan for right arm
void place(moveit::planning_interface::MoveGroup &right_arm_group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped place_object;
  place_object.header.frame_id = "base_footprint";
  place_object.pose.position.x = 0.7;
  place_object.pose.position.y = 0.0;
  place_object.pose.position.z = 0.5;
  place_object.pose.orientation.x = 0;
  place_object.pose.orientation.y = 0;
  place_object.pose.orientation.z = 0;
  place_object.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation grasp_place;
  grasp_place.place_pose = place_object;

  grasp_place.pre_place_approach.direction.vector.z = -1.0;
  grasp_place.post_place_retreat.direction.vector.x = -1.0;
  grasp_place.post_place_retreat.direction.header.frame_id = "base_footprint";
  grasp_place.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
  grasp_place.pre_place_approach.min_distance = 0.1;
  grasp_place.pre_place_approach.desired_distance = 0.2;
  grasp_place.post_place_retreat.min_distance = 0.1;
  grasp_place.post_place_retreat.desired_distance = 0.25;

  openGripper(grasp_place.post_place_posture);

  loc.push_back(grasp_place);
  right_arm_group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = place_object.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //  right_arm_group.setPathConstraints(constr);
  // right_arm_group.setPlannerId("RRTConnectkConfigDefault");

  right_arm_group.place("object", loc);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create a node handle for this node to communicate with other nodes
  ros::NodeHandle nh;
  // Advertise on topic at rate
  ros::Publisher col_obj_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher attached_col_obj_pub = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup right_arm_group("right_arm");
  // Planning time is high to plan the path in a collision environment
  right_arm_group.setPlanningTime(45.0);

  // collision_obj is for table and pole
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();

  // base_footprint is a virtual joint as a child link for the parent frame /odom_combined
  // It represents the motion of the base of the robot in a plane.
  collision_obj.header.frame_id = "base_footprint";

  // remove pole
  collision_obj.id = "pole";
  collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
  col_obj_pub.publish(collision_obj);

  // add pole
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0].position.x = 0.7;
  collision_obj.primitive_poses[0].position.y = -0.4;
  collision_obj.primitive_poses[0].position.z = 0.85;
  collision_obj.primitive_poses[0].orientation.w = 1.0;
  col_obj_pub.publish(collision_obj);



  // remove table
  collision_obj.id = "table";
  collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
  col_obj_pub.publish(collision_obj);

  // add table
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  collision_obj.primitive_poses[0].position.x = 0.7;
  collision_obj.primitive_poses[0].position.y = -0.2;
  collision_obj.primitive_poses[0].position.z = 0.175;
  col_obj_pub.publish(collision_obj);

  // Add object
  collision_obj.id = "object";
  collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
  col_obj_pub.publish(collision_obj);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = collision_obj;
  attached_col_obj_pub.publish(aco);

  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.25;

  collision_obj.primitive_poses[0].position.x = 0.6;
  collision_obj.primitive_poses[0].position.y = -0.7;
  collision_obj.primitive_poses[0].position.z = 0.5;
  col_obj_pub.publish(collision_obj);

  right_arm_group.setPlannerId("RRTConnectkConfigDefault");

  // wait a bit for ros things to initialize
  ros::WallDuration(5.0).sleep();

  pick(right_arm_group);

  ros::WallDuration(5.0).sleep();

  place(right_arm_group);

  ros::waitForShutdown();
  return 0;
}
