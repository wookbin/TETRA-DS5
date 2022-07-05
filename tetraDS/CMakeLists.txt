cmake_minimum_required(VERSION 2.8.3)
project(tetraDS)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  roscpp
  message_generation
  message_runtime
  tf
  nav_msgs
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  parameter_read.srv
  parameter_write.srv
  set_move_mode.srv
  linear_position_move.srv
  angular_position_move.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tetraDS
  CATKIN_DEPENDS roscpp message_generation message_runtime
)

###########
## Build ##
###########
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

add_executable(tetraDS include/rs232.c include/rs232.h include/rs232_common.c include/rs232_common.h src/drive_module.c src/drive_module.h src/dssp_rs232_drive_module.c src/dssp_rs232_drive_module.h src/tetraDS.cpp)

add_dependencies(tetraDS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetraDS ${catkin_LIBRARIES})
