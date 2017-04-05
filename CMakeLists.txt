cmake_minimum_required(VERSION 2.8.3)
project(costmap)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES costmap2d
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(costmap2d include/${PROJECT_NAME}/costmap.cpp include/${PROJECT_NAME}/cnmap.cpp)

install(TARGETS costmap2d
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

