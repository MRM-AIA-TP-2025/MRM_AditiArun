cmake_minimum_required(VERSION 3.8)
project(cashier_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/Bill.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Add executables
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp)

add_executable(printer_node src/printer_node.cpp)
ament_target_dependencies(printer_node rclcpp)

# Link against generated interfaces
rosidl_target_interfaces(publisher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(subscriber_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(printer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  publisher_node
  subscriber_node
  printer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()

