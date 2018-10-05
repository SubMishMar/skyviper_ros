cmake_minimum_required(VERSION 2.8.3)
project(pose_publisher)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  tf
  nav_msgs
  geometry_msgs
)

catkin_package()
find_package(OpenCV 3.3.1 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/pose_publisher.cpp src/pose_publisher_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )
