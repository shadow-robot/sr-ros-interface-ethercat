cmake_minimum_required(VERSION 2.8.3)
project(sr_tactile_sensor_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  realtime_tools
  ros_ethercat_model
  roscpp
  sr_robot_lib
  sr_robot_msgs
  sr_hardware_interface
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/sr_tactile_sensor_controller.cpp)
add_dependencies(sr_tactile_sensor_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Declare catkin package
catkin_package(
CATKIN_DEPENDS controller_interface sr_hardware_interface pluginlib sr_robot_msgs realtime_tools ros_ethercat_model sr_robot_lib
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
)

install(TARGETS sr_tactile_sensor_controller 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES sr_tactile_sensor.launch  sr_tactile_sensor_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


