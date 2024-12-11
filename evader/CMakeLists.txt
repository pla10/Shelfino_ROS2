cmake_minimum_required(VERSION 3.8)
project(evader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

add_executable(evader 
  src/evader.cpp
)

target_include_directories(evader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(evader ${Python3_LIBRARIES})


ament_target_dependencies(evader 
  rclcpp 
  rclcpp_action
  std_srvs
  geometry_msgs
  nav2_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  evader
  DESTINATION lib/${PROJECT_NAME})

ament_package()
