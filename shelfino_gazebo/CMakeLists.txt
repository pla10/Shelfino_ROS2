cmake_minimum_required(VERSION 3.5)
project(shelfino_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(destroy_shelfino src/destroy_shelfino.cpp)

target_compile_features(destroy_shelfino PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(destroy_shelfino
  rclcpp
  gazebo_msgs
  geometry_msgs
)

install(TARGETS
  destroy_shelfino
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
