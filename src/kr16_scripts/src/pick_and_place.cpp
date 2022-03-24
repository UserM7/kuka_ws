#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

/* #include <moveit_visual_tools/moveit_visual_tools.h>  This has not been ported to ros2 yet */
#include <rviz_visual_tools/rviz_visual_tools.hpp>
/* this is a standin for moveit_visual_tools visual_tools.prompt */
#include <moveit/macros/console_colors.h>

#include "rclcpp_action/rclcpp_action.hpp"
#include <control_msgs/msg/gripper_command.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include <math.h>

using GripperCommand = control_msgs::action::GripperCommand;
using Find = grasping_msgs::action::FindGraspableObjects;

void prompt(const std::string& message)
{
  printf(MOVEIT_CONSOLE_COLOR_GREEN "\n%s" MOVEIT_CONSOLE_COLOR_RESET, message.c_str());
  fflush(stdout);
  while (std::cin.get() != '\n' && rclcpp::ok())
    ;
}

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


  //ActionClient Code
  //
  auto action_client = rclcpp_action::create_client<GripperCommand>(move_group_node, "gripper_controller/gripper_cmd");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(move_group_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate open and close goals
  auto goal_open = GripperCommand::Goal();
  goal_open.command.position = 0.05;
  goal_open.command.max_effort = 0.0;

  auto goal_close = GripperCommand::Goal();
  goal_close.command.position = 0.015;
  goal_close.command.max_effort = 0.0;

  //Perception Client Code
  //
  auto action_client2 = rclcpp_action::create_client<Find>(move_group_node, "find_objects");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(move_group_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  auto goal_msg = Find::Goal();
  goal_msg.plan_grasps = false;

  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "ur_manipulator";

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

  move_group.setPlanningTime(15.0);
  RCLCPP_INFO(LOGGER, "Planning Time: %f", move_group.getPlanningTime());

  RCLCPP_INFO(LOGGER, "End effector pose: %s", move_group.getCurrentPose().pose);

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Start the Pick & Place Demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  
  prompt("Press 'Enter' to START the Pick & Place Demo");

  //
  // Plan to Home Position
  // ^^^^^^^^^^^^^^^^^^^^^^^
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  //joint_group_positions[0] = 0.00;  // Shoulder Pan
  joint_group_positions[1] = -2.50;  // Shoulder Lift
  joint_group_positions[2] = 1.50;  // Elbow
  joint_group_positions[3] = -1.50;  // Wrist 1
  joint_group_positions[4] = -1.55;  // Wrist 2
  //joint_group_positions[5] = 0.00;  // Wrist 3

  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  //  Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  prompt("Press 'Enter' to move to the Home position");

  //
  // Move to Home Position
  // ^^^^^^^^^^^^^^^^^^^^^^^

  move_group.execute(my_plan);


  prompt("Press 'Enter' to execute Perception");


  //
  // Get Pick Position
  // ^^^^^^^^^^^^^^^^^^^^^^

  auto goal_handle_future2 = action_client2->async_send_goal(goal_msg);

  rclcpp_action::ClientGoalHandle<Find>::SharedPtr goal_handle2 = goal_handle_future2.get();
  if (!goal_handle2) {
    RCLCPP_ERROR(move_group_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future2 = action_client2->async_get_result(goal_handle2);

  RCLCPP_INFO(move_group_node->get_logger(), "Waiting for result");

  rclcpp_action::ClientGoalHandle<Find>::WrappedResult wrapped_result2 = result_future2.get();

  switch (wrapped_result2.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was aborted");
      return 1;
      //break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was canceled");
      return 1;
      //break;
    default:
      RCLCPP_ERROR(move_group_node->get_logger(), "Unknown result code");
      return 1;
      //break;
  }

  RCLCPP_INFO(move_group_node->get_logger(), "result received");
  float pos_x = wrapped_result2.result->objects[1].object.primitive_poses[0].position.x;
  float pos_y = wrapped_result2.result->objects[1].object.primitive_poses[0].position.y;
  float round_x = roundf(pos_x * 100) / 100;
  float round_y = roundf(pos_y * 100) / 100;
  RCLCPP_INFO(move_group_node->get_logger(), "X: %f", round_x);
  RCLCPP_INFO(move_group_node->get_logger(), "Y: %f", round_y);

  prompt("Press 'Enter' to plan Pick position");

  // Planning to Pick Position
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose target_pose1;

  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = round_x;
  target_pose1.position.y = round_y;
  target_pose1.position.z = 0.284;
  move_group.setPoseTarget(target_pose1);
  //move_group.setOrientationTarget(-0.707, 0.707, 0.011, -0.004, "tool0");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualize the plan
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  prompt("Press 'Enter' to move to Pick position");
  /* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */

  //
  // Move to Pick Position
  // ^^^^^^^^^^^^^^^^^^^^^^^

  move_group.execute(my_plan);

  prompt("Press 'Enter' to Open Gripper");

  //
  // Send Open Goal
  // ^^^^^^^^^^^^^^^^^^^^^^^

  RCLCPP_INFO(move_group_node->get_logger(), "Sending open goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto goal_handle_future = action_client->async_send_goal(goal_open);

  rclcpp_action::ClientGoalHandle<GripperCommand>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(move_group_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(move_group_node->get_logger(), "Waiting for result");

  rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was aborted");
      return 1;
      //break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was canceled");
      return 1;
      //break;
    default:
      RCLCPP_ERROR(move_group_node->get_logger(), "Unknown result code");
      return 1;
      //break;
  }

  RCLCPP_INFO(move_group_node->get_logger(), "result received");

  prompt("Press 'Enter' to plan Approach");

  // Approach
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  float delta = 0.04;
  target_pose1.position.z = target_pose1.position.z - delta;
  move_group.setPoseTarget(target_pose1);
  //move_group.setOrientationTarget(-0.707, 0.707, 0.011, -0.004, "tool0");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualize the plan
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  prompt("Press 'Enter' to execute Approach");
  /* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */

  //
  // Execute Approach
  // ^^^^^^^^^^^^^^^^^^^^^^^

  move_group.execute(my_plan);

  prompt("Press 'Enter' to Close Gripper");

  //
  // Send Close Goal
  // ^^^^^^^^^^^^^^^^^^^^^^^

  RCLCPP_INFO(move_group_node->get_logger(), "Sending close goal");
  // Ask server to achieve some goal and wait until it's accepted
  goal_handle_future = action_client->async_send_goal(goal_close);

  goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(move_group_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(move_group_node->get_logger(), "Waiting for result");

  wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was aborted");
      return 1;
      //break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was canceled");
      return 1;
      //break;
    default:
      RCLCPP_ERROR(move_group_node->get_logger(), "Unknown result code");
      return 1;
      //break;
  }

  RCLCPP_INFO(move_group_node->get_logger(), "result received");

  prompt("Press 'Enter' to plan Retreat");

  // Retreat
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  target_pose1.position.z = target_pose1.position.z + delta;
  move_group.setPoseTarget(target_pose1);
  //move_group.setOrientationTarget(-0.707, 0.707, 0.011, -0.004, "tool0");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualize the plan
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  prompt("Press 'Enter' to execute Retreat");
  /* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */

  //
  // Execute Retreat
  // ^^^^^^^^^^^^^^^^^^^^^^^

  move_group.execute(my_plan);

  prompt("Press 'Enter' to plan Place position");


  // Planning to Place Position
  // ^^^^^^^^^^^^^^^^^^^^^^^
  target_pose1.position.x = 0.139;
  target_pose1.position.y = 0.340;
  move_group.setPoseTarget(target_pose1);
  //move_group.setOrientationTarget(-0.707, 0.707, 0.011, -0.004, "tool0");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualize the plan
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  /* visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); */
  visual_tools.trigger();

  prompt("Press 'Enter' to move to Place position");
  /* visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo"); */

  //
  // Move to Place Position
  // ^^^^^^^^^^^^^^^^^^^^^^^

  move_group.execute(my_plan);

  prompt("Press 'Enter' to Open Gripper");

  //
  // Send Open Goal
  // ^^^^^^^^^^^^^^^^^^^^^^^

  RCLCPP_INFO(move_group_node->get_logger(), "Sending open goal");
  // Ask server to achieve some goal and wait until it's accepted
  goal_handle_future = action_client->async_send_goal(goal_open);

  goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(move_group_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(move_group_node->get_logger(), "Waiting for result");

  wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was aborted");
      return 1;
      //break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(move_group_node->get_logger(), "Goal was canceled");
      return 1;
      //break;
    default:
      RCLCPP_ERROR(move_group_node->get_logger(), "Unknown result code");
      return 1;
      //break;
  }

  RCLCPP_INFO(move_group_node->get_logger(), "result received");

  prompt("Press 'Enter' to Finish Demo");

  rclcpp::shutdown();
  return 0;
}