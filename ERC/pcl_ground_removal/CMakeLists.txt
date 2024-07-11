cmake_minimum_required(VERSION 3.8)
project(pcl_ground_removal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(ground_removal_node src/ground_removal_node.cpp)
target_link_libraries(ground_removal_node ${PCL_LIBRARIES})
ament_target_dependencies(ground_removal_node rclcpp sensor_msgs pcl_conversions PCL)

install(TARGETS
  ground_removal_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

