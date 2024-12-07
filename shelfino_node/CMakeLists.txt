cmake_minimum_required(VERSION 3.8)
project(shelfino_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_executable(shelfino_node 
  src/shelfino_hw_publisher.cpp
  src/hardwareglobalinterface.cpp
  src/Subscriber.cpp
  src/Publisher.cpp
  src/RequesterSimple.cpp
  src/Replier.cpp
)
ament_target_dependencies(shelfino_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  std_srvs
)

target_link_libraries(shelfino_node
  ${rclcpp_LIBRARIES} -lzmq
)

install(DIRECTORY 
        launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  shelfino_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()