cmake_minimum_required(VERSION 3.2)
project(rplidar_reader)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(include/zmq_layer/)
include_directories(lib)
include_directories(lib/rplidar_sdk)
include_directories(lib/rplidar_sdk/include)
include_directories(lib/rplidar_sdk/src)
include_directories(lib/rplidar_sdk/src/arch)
include_directories(lib/rplidar_sdk/src/hal)
include_directories(lib/rplidar_sdk/driver/include)



add_executable(rplidar_reader
        lib/rplidar_sdk/include/rplidar.h
        lib/rplidar_sdk/include/rplidar_cmd.h
        lib/rplidar_sdk/include/rplidar_driver.h
        lib/rplidar_sdk/include/rplidar_protocol.h
        lib/rplidar_sdk/include/rptypes.h
        include/zmq_layer/Publisher.hpp
        include/zmq_layer/zmq.hpp
        include/json.hpp
        include/zmq_layer/zhelpers.hpp
        source/zmq_layer/Publisher.cpp
        source/main.cpp)
target_link_libraries(rplidar_reader ${CMAKE_CURRENT_SOURCE_DIR}/lib/rplidar_sdk/librplidar_sdk.a)
target_link_libraries(rplidar_reader pthread zmq)
set(CMAKE_HEADER_FILE include/zmq_layer/*)
