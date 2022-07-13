file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/VERSION version)
message(STATUS "Read version from file: ${version}")

cmake_minimum_required(VERSION 3.5.1)
project(virtual_costmap_layer2 LANGUAGES C CXX VERSION ${version})

set(CMAKE_CXX_STANDARD 14)
set(CXX_STANDARD_REQUIRED ON)


find_package(catkin
             REQUIRED
             roscpp
             costmap_2d
             dynamic_reconfigure
             tf)

generate_dynamic_reconfigure_options(cfg/VirtualLayer2.cfg)

add_message_files(DIRECTORY
                  msg
                  FILES
                  Form2.msg
                  Obstacles2.msg
                  Zone2.msg)
generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs
                  nav_msgs)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               virtual_layer2
               CATKIN_DEPENDS
               costmap_2d
               dynamic_reconfigure
               message_runtime
               tf
               DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(virtual_layer2 src/virtual_layer2.cpp)
target_link_libraries(virtual_layer2 ${catkin_LIBRARIES})
add_dependencies(virtual_layer2 ${PROJECT_NAME}_gencfg)
