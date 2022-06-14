cmake_minimum_required(VERSION 3.1.2)
project(mrs_actionlib_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mavros_msgs
  mrs_lib
  genmsg
  actionlib_msgs
  )

add_message_files(
  FILES
  State.msg
  )

add_action_files(DIRECTORY action FILES command.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(mrs_actionlib_interface src/mrs_actionlib_interface.cpp)
add_dependencies(mrs_actionlib_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mrs_actionlib_interface
  ${catkin_LIBRARIES}
)
