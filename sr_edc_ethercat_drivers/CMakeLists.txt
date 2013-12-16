# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_edc_ethercat_drivers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sr_robot_lib ethercat_hardware sr_external_dependencies message_generation)
find_package(binutils REQUIRED)
find_package(Boost REQUIRED COMPONENTS smart_ptr algorithm assign)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)

include_directories(include ${binutils_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${ImageMagick_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#uncomment if you have defined messages
add_message_files(
  FILES
  ActuatorInfo.msg
  MotorTrace.msg
  MotorTraceSample.msg
)
#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)


add_library(sr_edc_ethercat_drivers src/sr0x.cpp src/sr_edc.cpp src/sr06.cpp src/sr08.cpp src/sr_edc_muscle.cpp src/srbridge.cpp src/motor_trace_buffer.cpp)
target_link_libraries(sr_edc_ethercat_drivers bfd)
add_dependencies(sr_edc_ethercat_drivers ${PROJECT_NAME}_generate_messages_cpp)


###############
#   EXAMPLES  #
###############
add_executable(examples/send_targets examples/send_targets.cpp)

###############
#    TESTS    #
###############
add_executable(test_clock test/test_clock_resolution.cpp)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(test_clock ${Boost_LIBRARIES})
target_link_libraries (test_clock rt)

## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS binutils
    CATKIN_DEPENDS roscpp roslib std_msgs sr_robot_lib ethercat_hardware sr_external_dependencies message_runtime
    INCLUDE_DIRS include
    LIBRARIES # TODO
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS sr_edc_ethercat_drivers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
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




