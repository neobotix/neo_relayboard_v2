cmake_minimum_required(VERSION 2.8.3)
project(neo_relayboard_v2)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  neo_msgs
  neo_srvs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neo_relayboard
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/RelayBoardClient.cpp common/src/NeoRelayBoardNode.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(neo_relayboard_v2_node src/neo_relayboard_v2_node.cpp)
target_link_libraries(neo_relayboard_v2_node ${PROJECT_NAME})
target_link_libraries(neo_relayboard_v2_node ${catkin_LIBRARIES})
add_dependencies(neo_relayboard_v2_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############



#############
## Testing ##
#############


