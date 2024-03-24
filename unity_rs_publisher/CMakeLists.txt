cmake_minimum_required(VERSION 3.8)
project(unity_rs_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(unity_rs_publisher_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(unity_rs_publisher src/main.cpp)
ament_target_dependencies(unity_rs_publisher unity_rs_publisher_msgs rclcpp image_transport cv_bridge std_msgs sensor_msgs)

install(TARGETS
  unity_rs_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
