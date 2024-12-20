cmake_minimum_required(VERSION 3.8)
project(shelfino_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclpy REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(mapinfo_node src/mapinfo.cpp)
ament_target_dependencies(mapinfo_node rclcpp geometry_msgs obstacles_msgs)

add_executable(shelfino_dates src/shelfino_dates.cpp)
ament_target_dependencies(shelfino_dates rclcpp geometry_msgs nav_msgs obstacles_msgs )

add_executable(samples_marker_node src/prm/samples_marker.cpp)
ament_target_dependencies(samples_marker_node rclcpp geometry_msgs visualization_msgs)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  shelfino_dates
  mapinfo_node
  samples_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/py_code/creation_of_sample.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
