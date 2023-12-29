cmake_minimum_required(VERSION 3.8)
project(map_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

install(DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

# list of nodes
set(NODES
  send_borders
  send_gates
  send_obstacles
  send_victims
)

add_library(${PROJECT_NAME}_lib SHARED
  src/utilities.cpp
  src/spawn_model.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ${PROJECT_NAME}_lib
  "geometry_msgs"
  "rclcpp"
  "gazebo_msgs"
  "obstacles_msgs"
  "tf2"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
)

# build nodes
foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp src/utilities.cpp src/spawn_model.cpp)
  
  target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(${NODE} PUBLIC c_std_99 cxx_std_17)
  
  # target_link_libraries(${NODE} ${PROJECT_NAME}_lib)

  ament_target_dependencies(
    ${NODE}
    "geometry_msgs"
    "rclcpp"
    "gazebo_msgs"
    "obstacles_msgs"
    "tf2"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
  )

  install(TARGETS ${NODE}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# ament_python_install_package(scripts)

install(PROGRAMS
  scripts/create_map_pgm.py
  DESTINATION lib/${PROJECT_NAME}
)


# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_tests test/utilities_test.cpp)

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_tests
    "geometry_msgs"
    "rclcpp"
    "std_msgs"
    "gazebo_msgs"
    "obstacles_msgs"
  )
  
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}_lib)
endif()

ament_package()
