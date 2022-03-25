cmake_minimum_required(VERSION 3.0.2)
project(autumn_pathfinding_2d)
include_directories(include)
include_directories(../spdlog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(explorator src/main.cpp src/PathPlanning.cpp)
target_link_libraries(explorator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
