cmake_minimum_required(VERSION 2.8.3)
project(gazebo_model_destroyer_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS 
    roscpp 
    message_generation
    gazebo_ros
)

find_package(gazebo REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
    Destroy.srv
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gazebo_model_destroyer_plugin
  CATKIN_DEPENDS
    roscpp
    message_runtime
    gazebo_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gazebo_model_destroyer_plugin src/gazebo_model_destroyer_plugin.cpp)
add_dependencies(gazebo_model_destroyer_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_model_destroyer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS 
    gazebo_model_destroyer_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
