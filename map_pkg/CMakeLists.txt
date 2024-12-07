cmake_minimum_required(VERSION 3.8)
project(map_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

# add_compile_options(-DHOMOG2D_NOWARNINGS)

install(DIRECTORY launch models config
  DESTINATION share/${PROJECT_NAME}
)

set(CMAKE_CXX_STANDARD 17)

# list of nodes
set(NODES
  send_borders
  send_gates
  send_obstacles
  send_victims
)

# build nodes
foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp src/utilities.cpp)

  target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(${NODE} PUBLIC c_std_99 cxx_std_17)

  target_link_libraries(${NODE} ${Python3_LIBRARIES})
  # target_link_libraries(${NODE} ${PROJECT_NAME}_lib)

  ament_target_dependencies(
    ${NODE}
    "geometry_msgs"
    "rclcpp"
    "obstacles_msgs"
    "tf2"
    "tf2_ros"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "visualization_msgs"
  )

  install(TARGETS ${NODE}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# ament_python_install_package(scripts)

install(PROGRAMS
  scripts/create_map_pgm.py
  scripts/generate_config_file.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
