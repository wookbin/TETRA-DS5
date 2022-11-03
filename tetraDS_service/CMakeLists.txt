cmake_minimum_required(VERSION 2.8.3)
project(tetraDS_service)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
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
  getinformation.srv
  dockingcontrol.srv
  getlocationlist.srv
  getlandmarklist.srv
  deletelocation.srv
  deletelandmark.srv
  deletemap.srv
  runmapping.srv
  runnavigation.srv
  rosnodekill.srv
  getmaplist.srv
  ledcontrol.srv
  ledtogglecontrol.srv
  gotocancel.srv
  setmaxspeed.srv
  toggleon.srv
  accelerationslop.srv
  servo.srv
  virtual_obstacle.srv
  setinitpose.srv
  patrol.srv
  gotoconveyor.srv
  loadingcheck.srv
  unloadingcheck.srv
  deletedataall.srv
  patrol_conveyor.srv
  conveyor_auto_movement.srv
  pose_estimate.srv
  all_data_reset.srv
  euler_angle_init.srv
  euler_angle_reset.srv
  pose_velocity_reset.srv
  reboot_sensor.srv
  SetPose.srv
  sethome_id.srv
  setekf.srv
  power_sonar_cmd.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

catkin_package(
  LIBRARIES tetraDS_service
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_generation message_runtime
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(tetraDS_service src/tetraDS_service.cpp)
add_dependencies(tetraDS_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetraDS_service ${catkin_LIBRARIES})
