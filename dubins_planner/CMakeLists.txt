cmake_minimum_required(VERSION 3.8)
project(dubins_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(dubins_planner 
  src/path_planner.cpp
)
ament_target_dependencies(dubins_planner 
  rclcpp 
  std_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  rclcpp_components
  rclcpp_action
)

install(DIRECTORY 
        launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  dubins_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
