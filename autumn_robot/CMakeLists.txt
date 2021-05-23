cmake_minimum_required(VERSION 3.0.2)
project(autumn_robot)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package()

FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})


#############
## Install ##
#############

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
