cmake_minimum_required(VERSION 3.8)
project(unity_rs_publisher)

# Read ROS_DISTRO and if it is "humble", define ROS_HUMBLE.
if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_HUMBLE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(unity_rs_publisher_msgs REQUIRED)

add_library(unity_rs_publisher_node SHARED src/unity_rs_publisher_node.cpp)
ament_target_dependencies(unity_rs_publisher_node rclcpp rclcpp_components image_transport cv_bridge std_msgs sensor_msgs unity_rs_publisher_msgs)
rclcpp_components_register_node(
  unity_rs_publisher_node
  PLUGIN "unity_rs_publisher::UnityRsPublisher"
  EXECUTABLE unity_rs_publisher
)

ament_export_targets(unity_rs_publisher-targets)
install(TARGETS unity_rs_publisher_node
  EXPORT unity_rs_publisher-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
