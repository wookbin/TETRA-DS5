cmake_minimum_required(VERSION 2.8.3)
project(tetraDS_TCP)

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
  visualization_msgs 
  dynamic_reconfigure
)

add_service_files(
  FILES
  gotolocation.srv
  gotolocation2.srv
  getlocation.srv
  setlocation.srv
  setsavemap.srv
  dockingcontrol.srv
  getlocationlist.srv
  deletelocation.srv
  runmapping.srv
  runnavigation.srv
  rosnodekill.srv
  getmaplist.srv
  gotocancel.srv
  setmaxspeed.srv
  power_set_outport.srv
  power_get_io_status.srv
  power_set_single_outport.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

catkin_package(
  LIBRARIES ros_server
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_generation message_runtime
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
