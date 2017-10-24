# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_edc_ethercat_drivers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sr_robot_lib ros_ethercat_hardware sr_robot_msgs sr_external_dependencies message_generation)
find_package(Boost REQUIRED COMPONENTS)

find_library(BFD_LIBRARY bfd)

message(STATUS "Found libbfd: ${BFD_LIBRARY}")

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_message_files(
        FILES
        ActuatorInfo.msg
        MotorTrace.msg
        MotorTraceSample.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
        DEPENDS bfd
        CATKIN_DEPENDS roscpp roslib std_msgs sr_robot_lib ros_ethercat_hardware sr_external_dependencies message_runtime
        INCLUDE_DIRS include
        LIBRARIES sr_edc_ethercat_drivers
)


add_library(sr_edc_ethercat_drivers src/sr0x.cpp src/sr_edc.cpp src/sr06.cpp src/sr08.cpp src/sr09.cpp src/sr_edc_muscle.cpp src/srbridge.cpp src/motor_trace_buffer.cpp)
add_dependencies(sr_edc_ethercat_drivers ${sr_edc_ethercat_drivers_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_edc_ethercat_drivers ${BFD_LIBRARY} ${Boost_LIBRARIES} ${catkin_LIBRARIES})



###############
#   EXAMPLES  #
###############
#add_executable(send_targets examples/send_targets.cpp)

###############
#    TESTS    #
###############
add_executable(test_clock test/test_clock_resolution.cpp)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(test_clock ${Boost_LIBRARIES})
target_link_libraries(test_clock rt)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS sr_edc_ethercat_drivers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        scripts/flash_all_motors.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY examples/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)

install(FILES ethercat_device_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
