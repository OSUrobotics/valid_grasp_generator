cmake_minimum_required(VERSION 2.8.3)
project(valid_grasp_generator)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(Boost_USE_STATIC_LIBS OFF)  
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    sensor_msgs
    )

find_package(OpenRAVE REQUIRED)
find_package(Boost COMPONENTS
    system
    thread
    python
    REQUIRED)

add_message_files(
   FILES
   GraspSnapshot.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs  # Or other packages containing msgs
 )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Valid_grasp_generator
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  "/usr/include/python2.7"
  "-lboost_python"
)
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${PYTHON_LIBRARIES})

## Specify libraries to link a library or executable target against

add_library(depth_penetration SHARED src/depth_penetration.cpp)
target_link_libraries(depth_penetration ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenRAVE_LIBRARIES})

