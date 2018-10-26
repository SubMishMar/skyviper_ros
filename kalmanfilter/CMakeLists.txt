cmake_minimum_required(VERSION 2.8.3)
project(kalmanfilter)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
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

add_executable(${PROJECT_NAME}_node src/kalmanfilter.cpp src/kalmanfilter_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
 )
