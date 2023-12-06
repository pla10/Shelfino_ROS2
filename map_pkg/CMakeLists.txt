cmake_minimum_required(VERSION 3.8)
project(map_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# list of nodes
set(NODES
  send_borders
  send_gates
)

# build nodes
foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp)
  target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${NODE} PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(
    ${NODE}
    "geometry_msgs"
    "rclcpp"
    "std_msgs"
    "gazebo_msgs"
  )

  install(TARGETS ${NODE}
    DESTINATION lib/${PROJECT_NAME})
endforeach()


# If the WITHOUT_OBSTACLE variable is defined, then DO NOT  build the obstacles node
if(NOT DEFINED WITHOUT_OBSTACLE)
  find_package(obstacles_msgs REQUIRED)
  
  add_executable(send_obstacles src/send_obstacles.cpp)
  
  target_include_directories(send_obstacles PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(send_obstacles PUBLIC c_std_99 cxx_std_17)
  
  ament_target_dependencies(
    send_obstacles
    "geometry_msgs"
    "rclcpp"
    "std_msgs"
    "obstacles_msgs"
    "gazebo_msgs"
  )

  install(TARGETS send_obstacles
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
