cmake_minimum_required(VERSION 2.8.3)
project(torso_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotnik_msgs
  rcomponent
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES torso_interface
 CATKIN_DEPENDS roscpp rcomponent robotnik_msgs 
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(torso_interface
  src/torso_interface.cpp
)

add_dependencies(torso_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(torso_interface_node src/torso_interface_node.cpp)

add_dependencies(torso_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(torso_interface ${catkin_LIBRARIES})

target_link_libraries(torso_interface_node
  ${catkin_LIBRARIES}
  torso_interface
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS torso_interface torso_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
