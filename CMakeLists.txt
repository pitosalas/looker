cmake_minimum_required(VERSION 3.8)
project(looker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(costmap_node src/costmap_node.cpp)
target_include_directories(costmap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(costmap_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  costmap_node
  rclcpp
  nav2_costmap_2d
  tf2_ros
)

# Install
install(TARGETS costmap_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()