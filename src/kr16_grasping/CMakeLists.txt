cmake_minimum_required(VERSION 3.8)
project(kr16_grasping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# find dependencies
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(control_msgs REQUIRED)
#find_package(moveit_msgs REQUIRED)
#find_package(grasping_msgs REQUIRED)


add_executable(grasping src/grasping.cpp)
ament_target_dependencies(grasping  ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
rviz_visual_tools)

	
add_executable(grasp src/test_grasp.cpp)
ament_target_dependencies(grasp  ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
rviz_visual_tools)

add_executable(pick_place src/pick_place.cpp)
ament_target_dependencies(pick_place  ament_cmake
rclcpp
rclcpp_action
moveit_core
moveit_ros_planning_interface
rviz_visual_tools)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

install(DIRECTORY src launch scripts DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  grasping
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  grasp
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
