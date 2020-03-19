cmake_minimum_required(VERSION 2.8.3)
project(surfr_core)

## Check ROS distro
set(ROS_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS_DISTRO $ENV{ROS_DISTRO})
  set(ROS_FOUND TRUE)
else()
  message("ROS distro variable not set. Trying to figure it out...")
  set(AVAILABLE_ROS_VERSIONS "hydro;indigo;kinetic;melodic")
  set(ROS_FOUND FALSE)
  foreach(version ${AVAILABLE_ROS_VERSIONS})
    if(NOT ROS_FOUND)
      find_path(ROS_H ros.h PATHS /opt/ros/${version}/include/ros)
      if(ROS_H)
        message("Found ros version ${version}")
        set(FOUND_ROS_DISTRO ${version})
        set(ROS_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()
if(ROS_FOUND)
  if($ENV{ROS_DISTRO} STREQUAL "melodic")
    message("Using ROS ${FOUND_ROS_DISTRO}")
    ## To use C++14 standard (i.e. std::to_string()...)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    find_package(Eigen3 REQUIRED)
  elseif($ENV{ROS_DISTRO} STREQUAL "kinetic")
    message("Using ROS ${FOUND_ROS_DISTRO}")
    ## To use C++14 standard (i.e. std::to_string()...)
    set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    find_package(Eigen3 REQUIRED)
  elseif($ENV{ROS_DISTRO} STREQUAL "indigo")
    message("Using ROS ${FOUND_ROS_DISTRO}")
    ## To use C++11 standard (i.e. std::to_string()...)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    find_package(Eigen3 REQUIRED)
  elseif($ENV{ROS_DISTRO} STREQUAL "hydro")
    message("Using ROS ${FOUND_ROS_DISTRO}")
    ## To use C++0x standard (i.e. std::to_string()...)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
    find_package(Eigen REQUIRED)
  # ... check other versions ...
  else()
    message("Unknown ROS distro:")
    message(${FOUND_ROS_DISTRO})
  endif()
else()
  message("ROS distro is unknown.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  roslib
  rosbag
  nodelet
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  surfr_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
#     sfData
    sfCore
    sfBase
    sfFilters
    sfPlanners
    sfManipulator
    sfNode
    sfNodelet
    sfSerial
    sfEthernet
  CATKIN_DEPENDS roscpp rosconsole moveit_core moveit_ros_planning_interface surfr_msgs cv_bridge nodelet libntcan libpcan
  #DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
# add_library(sfData src/data.cpp)
add_library(sfCore        src/core/funcs.cpp)
add_library(sfBase        src/Base.cpp
                          src/StateMachine.cpp)
add_library(sfFilters     src/filters/Filter.cpp
                          src/filters/Iir2.cpp)
add_library(sfPlanners    src/planners/Planner.cpp
                          src/planners/Order1Linear.cpp
                          src/planners/Order1Angular.cpp
                          src/planners/Order1Pose.cpp)
add_library(sfManipulator src/dev/Manipulator.cpp)
add_library(sfNode        src/node/Node.cpp)
add_library(sfNodelet     src/node/Nodelet.cpp)
add_library(sfSerial      src/comm/Serial.cpp)
add_library(sfEthernet    src/comm/Ethernet.cpp)
# add_library(Camera src/dev/Camera.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(lib_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(main_device src/node/main_device.cpp)
add_executable(main_socket src/node/main_socket.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(exec_example ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sfCore        ${catkin_LIBRARIES})
target_link_libraries(sfBase        ${catkin_LIBRARIES})
target_link_libraries(sfFilters     ${catkin_LIBRARIES})
target_link_libraries(sfPlanners    ${catkin_LIBRARIES})
target_link_libraries(sfManipulator ${catkin_LIBRARIES})
target_link_libraries(sfNode        ${catkin_LIBRARIES}
                                    sfCore
                                    sfBase
                                    sfFilters
                                    sfPlanners
                                    sfManipulator)
target_link_libraries(sfNodelet     ${catkin_LIBRARIES} sfNode)
target_link_libraries(sfSerial      ${catkin_LIBRARIES})
target_link_libraries(sfEthernet    ${catkin_LIBRARIES})
# target_link_libraries(Camera       ${catkin_LIBRARIES} Base)
target_link_libraries(main_device  ${catkin_LIBRARIES} sfNode)
target_link_libraries(main_socket  ${catkin_LIBRARIES} sfNode)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS Data
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosmed_drivers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
