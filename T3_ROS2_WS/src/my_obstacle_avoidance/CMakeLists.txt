cmake_minimum_required(VERSION 3.5)
project(my_obstacle_avoidance)

# Default to C99 for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Executable: vfh_avoidance
add_executable(vfh_avoidance src/vfh_avoidance.cpp)
ament_target_dependencies(vfh_avoidance
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
  geometry_msgs
  PCL
  interfaces
)
target_link_libraries(vfh_avoidance ${PCL_LIBRARIES})
# rosidl_target_interfaces(vfh_avoidance
#   interfaces "rosidl_typesupport_cpp"
# )

# Executable: publish_command_pub
#add_executable(publish_command_pub src/publish_command_pub.cpp)
#ament_target_dependencies(publish_command_pub
 # rclcpp
  #sensor_msgs
  #interfaces
#)
#target_link_libraries(publish_command_pub ${PCL_LIBRARIES})
# # rosidl_target_interfaces(publish_command_pub
# #   interfaces "rosidl_typesupport_cpp"
# # )

# Install targets
install(TARGETS
  vfh_avoidance
  #publish_command_pub
  DESTINATION lib/${PROJECT_NAME}
)

# Export include directory
#install(DIRECTORY include/
#  DESTINATION include
#)

# Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install config files
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_package()
