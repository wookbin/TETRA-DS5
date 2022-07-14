cmake_minimum_required(VERSION 2.8.3)
project(tetraDS_landmark)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
  message_runtime
  tf
  geometry_msgs
  visualization_msgs 
)

add_service_files(
  FILES
  savemark.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

catkin_package(
  LIBRARIES tetraDS_landmark
  CATKIN_DEPENDS roscpp std_msgs std_srvs message_generation message_runtime
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(tetraDS_landmark_save src/tetraDS_landmark_save.cpp)
add_dependencies(tetraDS_landmark_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetraDS_landmark_save ${catkin_LIBRARIES})

add_executable(tetraDS_landmark_load src/tetraDS_landmark_load.cpp)
add_dependencies(tetraDS_landmark_load ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetraDS_landmark_load ${catkin_LIBRARIES})
