cmake_minimum_required(VERSION 2.8.3)
project(tetraDS_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  message_generation
  message_runtime
  tf
  nav_msgs
  std_srvs
  pcl_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_message_files(
	FILES 
	GPIO.msg
)

add_service_files(
  FILES
  ledcontrol.srv
  ledtogglecontrol.srv
  toggleon.srv
  Integrallog.srv
  power_set_outport.srv
  loadcell_callibration.srv
  conveyor_manual_movement.srv
  conveyor_auto_movement.srv
  power_set_enable.srv
  power_wheel_enable.srv
  power_parameter_read.srv
  conveyor_parameter_read.srv
  power_parameter_write.srv
  power_set_single_enable.srv
  power_set_single_outport.srv
  conveyor_parameter_write.srv
  conveyor_data_read.srv
  power_data_read.srv
  power_version_read.srv
  power_sonar_read.srv
  power_adc_read.srv
  power_sonar_cmd.srv
  power_get_io_status.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tetraDS_interface
  CATKIN_DEPENDS pcl_ros roscpp std_srvs std_msgs sensor_msgs tf2 tf2_geometry_msgs message_generation message_runtime
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(tetraDS_interface include/rs232.c include/rs232.h include/rs232_common.c include/rs232_common.h src/power_module.c src/power_module.h src/dssp_rs232_power_module.c src/dssp_rs232_power_module.h src/tetraDS_interface.cpp)

add_dependencies(tetraDS_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetraDS_interface ${catkin_LIBRARIES})
