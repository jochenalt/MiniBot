cmake_minimum_required(VERSION 2.8.3)
project(minibot)

## Compile as C++14 and with full debug information
add_compile_options(-std=c++14 -g2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  urdf
  tf
  eigen_conversions
  message_generation
  geometry_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
moveit_ros_planning
  moveit_ros_planning_interface
  visualization_msgs interactive_markers 
)

find_package(LAPACK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
    system thread program_options filesystem)
find_package(Boost REQUIRED system)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Statement.msg
   Programme.msg
   ErrorCodes.msg
   MinibotState.msg
   MinibotPose.msg
   PoseStorage.msg
   Configuration.msg
   JointStateConfiguration.msg
   ClientAction.msg
   LocalPlan.msg
   GlobalPlan.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DatabaseAction.srv
  PlanningAction.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
)

catkin_package(
   INCLUDE_DIRS
    include
   LIBRARIES
     ${PROJECT_NAME}
   CATKIN_DEPENDS 
     cmake_modules
     roscpp 
     message_runtime 
     geometry_msgs 
     sensor_msgs 
     std_msgs 
     move_base_msgs
     moveit_core
     moveit_ros_planning_interface
   DEPENDS 
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
   ../minibot_ikfast_plugin/include
)

add_executable(${PROJECT_NAME}_server 
   src/node.cpp
   src/kinematics.cpp
   src/utils.cpp
   src/planner.cpp
   src/dispatcher.cpp
   src/marker.cpp
   src/database.cpp
   src/message_store.cpp
   src/trap_velocity_profile.cpp
   src/trajectory_ptp.cpp
   src/trajectory_cartesian.cpp
   src/execution.cpp
   src/uniform_sample_filter.cpp
   src/globals.cpp
)

target_link_libraries(
   ${PROJECT_NAME}_server ${catkin_LIBRARIES}
   ${catkin_LIBRARIES}
   ${LAPACK_LIBRARIES} # required for IKFast
   ${Boost_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   $ENV{HOME}/mongo-client-install/lib/libmongoclient.a
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   scripts/planning.py
   scripts/constants.py
 )

install(
  TARGETS ${PROJECT_NAME}_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
