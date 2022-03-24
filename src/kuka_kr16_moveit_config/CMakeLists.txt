cmake_minimum_required(VERSION 3.10.2)
project(kuka_kr16_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
