cmake_minimum_required(VERSION 2.8.3)
project(ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -ffast-math")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    cv_bridge
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    )

catkin_package(
    )

add_executable(ekf
    src/ekf_node.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES} ${OpenCV_LIBS} )

