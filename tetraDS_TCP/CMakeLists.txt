cmake_minimum_required(VERSION 2.8.3)
project(tetraDS_TCP)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  std_msgs
  std_srvs
  message_generation
  message_runtime
  tf
  nav_msgs
  geometry_msgs
  move_base
  custom_msgs
  visualization_msgs 
  dynamic_reconfigure
)

generate_messages(
  DEPENDENCIES
 # std_msgs  
)

catkin_package(
  LIBRARIES ros_server
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_generation message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
)

#add_compile_options(-std=c++11)
#add_executable(tetra_ds4 include/tetra_ds4/timeout.cpp include/tetra_ds4/serial.cpp src/tetra_ds4.cpp src/use_api.cpp )
add_executable(tetraDS_TCP src/tetraDS_TCP.cpp)

add_dependencies(tetraDS_TCP ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetraDS_TCP ${catkin_LIBRARIES})

