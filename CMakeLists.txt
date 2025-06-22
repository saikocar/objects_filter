cmake_minimum_required(VERSION 3.5)
project(objects_filter)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)

add_executable(object_filter_node src/object_filter_node.cpp)

ament_target_dependencies(object_filter_node
  rclcpp
  autoware_auto_perception_msgs
  visualization_msgs
)

install(TARGETS
  object_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

