#include <rclcpp/rclcpp.hpp>
#include <memory>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/macros/console_colors.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


#include <rviz_visual_tools/rviz_visual_tools.hpp>


// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "kr16_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // MoveitVisualTools has not been ported to ROS2 yet so we make use of RvizVisualTools for visualization.
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.

  namespace rvt = rviz_visual_tools;
  rviz_visual_tools::RvizVisualTools visual_tools("base_link", "move_group_tutorial", move_group_node);
  /* moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0"); */
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  /* visual_tools.loadRemoteControl(); */

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setPoseReferenceFrame("base_link");
  RCLCPP_INFO(LOGGER, "Pose Reference Frame: %s", move_group.getPoseReferenceFrame().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  //
  // Plan to 1st Position
  // ^^^^^^^^^^^^^^^^^^^^^^^
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.00;  // Shoulder Pan
  joint_group_positions[1] = -1.57;  // Shoulder Lift
  joint_group_positions[2] = 0.00;  // Elbow
  joint_group_positions[3] = 0.00;  // Wrist 1
  joint_group_positions[4] = 0.00;  // Wrist 2
  joint_group_positions[5] = 0.00;  // Wrist 3

  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

  //  Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  //
  // Move to 1st Position
  // ^^^^^^^^^^^^^^^^^^^^^^^

  move_group.execute(my_plan);

  //
  // Plan & Execute to 2nd Position
  // ^^^^^^^^^^^^^^^^^^^^^^^

  joint_group_positions[0] = -1.57;  // Shoulder Pan
  joint_group_positions[1] = -1.57;  // Shoulder Lift
  joint_group_positions[2] = 1.57;  // Elbow
  joint_group_positions[3] = 0.00;  // Wrist 1
  joint_group_positions[4] = 0.00;  // Wrist 2
  joint_group_positions[5] = 1.57;  // Wrist 3

  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  //  Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}