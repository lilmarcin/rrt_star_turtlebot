cmake_minimum_required(VERSION 3.5.1)
project(rrt_miapr)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  grid_map_rviz_plugin
  rviz
  map_server
  nav_msgs
  rospy
  std_msgs
)

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
  #INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS
    grid_map_rviz_plugin
    rviz
    map_server
    nav_msgs
    rospy
    std_msgs
  #DEPENDS
)

#############
## Install ##
#############

install(DIRECTORY config doc launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

