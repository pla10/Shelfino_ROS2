cmake_minimum_required(VERSION 3.8)
project(graph_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(graph_msgs
  "msg/Edges.msg"
  "msg/GeometryGraph.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)
ament_package()
