cmake_minimum_required(VERSION 3.8)
project(unity_tf_static_republisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(unity_tf_static_republisher_node SHARED src/unity_tf_static_republisher_node.cpp)
ament_target_dependencies(unity_tf_static_republisher_node rclcpp rclcpp_components tf2_msgs)
rclcpp_components_register_node(
  unity_tf_static_republisher_node
  PLUGIN "unity_tf_static_republisher::UnityTFStaticRepublisher"
  EXECUTABLE unity_tf_static_republisher
)

ament_export_targets(unity_tf_static_republisher-targets)
install(TARGETS unity_tf_static_republisher_node
  EXPORT unity_tf_static_republisher-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
