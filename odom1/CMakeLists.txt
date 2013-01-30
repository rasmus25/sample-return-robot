cmake_minimum_required(VERSION 2.8.3)
project(odom1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include 
#  LIBRARIES odom1
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

execute_process(
OUTPUT_VARIABLE GAZEBO_DIR
COMMAND rospack find gazebo
)

string(STRIP ${GAZEBO_DIR} GAZEBO_DIR)

execute_process(
OUTPUT_VARIABLE GAZEBO_MSGS_DIR
COMMAND rospack find gazebo_msgs
)

string(STRIP ${GAZEBO_MSGS_DIR} GAZEBO_MSGS_DIR)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${GAZEBO_MSGS_DIR}/gazebo_msgs/srv_gen/cpp/include
  ${GAZEBO_DIR}/gazebo/include/gazebo-1.2/
  ${GAZEBO_DIR}/gazebo/include/gazebo-1.2/gazebo/
  ${GAZEBO_DIR}/gazebo/include/gazebo-1.2/gazebo/physics/
  ${GAZEBO_DIR}/gazebo/include/gazebo-1.2/gazebo/common/
 )

## Declare a cpp library
add_library(diffdrive_wheel_angles_plugin src/diffdrive_wheel_angles_plugin.cpp)
add_library(diffdrive_control_plugin src/diffdrive_control_plugin.cpp)

## Declare a cpp executable
# add_executable(odom1_node src/odom1_node.cpp)

## Add dependencies to the executable
# add_dependencies(odom1_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(diffdrive_wheel_angles_plugin
   ${catkin_LIBRARIES}
)

target_link_libraries(diffdrive_control_plugin
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS odom1 odom1_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_odom1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
