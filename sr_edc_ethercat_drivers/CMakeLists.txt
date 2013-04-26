cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories( ${ImageMagick_INCLUDE_DIRS})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


rosbuild_add_library(sr_edc_ethercat_drivers src/sr0x.cpp src/sr06.cpp src/srbridge.cpp src/motor_trace_buffer.cpp)
target_link_libraries(sr_edc_ethercat_drivers bfd)


###############
#   EXAMPLES  #
###############
rosbuild_add_executable(examples/send_targets examples/send_targets.cpp)

###############
#    TESTS    #
###############
rosbuild_add_executable(test_clock test/test_clock_resolution.cpp)
rosbuild_link_boost (test_clock thread system)
target_link_libraries (test_clock rt)

