cmake_minimum_required(VERSION 3.1.2)
project(mrs_actionlib_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_msgs
  mrs_lib
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MrsActionlibInterface
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(MrsActionlibInterface src/mrs_actionlib_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(MrsActionlibInterface
  ${catkin_LIBRARIES}
  )
