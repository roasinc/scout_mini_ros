cmake_minimum_required(VERSION 2.8.3)
project(scout_mini_lib)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  can_msgs
  socketcan_interface
  rosconsole_bridge
)

catkin_package(
  LIBRARIES scout_mini_controller scout_mini_hardware
  INCLUDE_DIRS include
  DEPENDS
  CATKIN_DEPENDS
)

add_library(scout_mini_controller
  src/scout_mini_can_bridge.cpp
  src/scout_mini_controller.cpp
)
target_link_libraries(scout_mini_controller ${catkin_LIBRARIES})

add_library(scout_mini_hardware
  src/scout_mini_hardware.cpp
)
target_link_libraries(scout_mini_hardware ${catkin_LIBRARIES})

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

install(
  FILES include/${PROJECT_NAME}/resource.h include/${PROJECT_NAME}/scout_mini_can_bridge.h include/${PROJECT_NAME}/scout_mini_controller.h include/${PROJECT_NAME}/scout_mini_hardware.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
)

install(
  TARGETS scout_mini_controller scout_mini_hardware
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)