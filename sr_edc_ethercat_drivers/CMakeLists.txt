# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_edc_ethercat_drivers)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sr_robot_lib ethercat_hardware sr_external_dependencies binutils)
find_package(Boost REQUIRED COMPONENTS smart_ptr algorithm assign)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${ImageMagick_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

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
  # TODO: List your msg files here
)
#uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)


add_library(sr_edc_ethercat_drivers src/sr0x.cpp src/sr_edc.cpp src/sr06.cpp src/sr08.cpp src/sr_edc_muscle.cpp src/srbridge.cpp src/motor_trace_buffer.cpp)
target_link_libraries(sr_edc_ethercat_drivers bfd)


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
    DEPENDS roscpp roslib std_msgs sr_robot_lib ethercat_hardware sr_external_dependencies binutils
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
