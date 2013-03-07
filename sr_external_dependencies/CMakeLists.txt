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

SET(internal_firmware $ENV{INTERNAL_FIRMWARE})
IF(internal_firmware)
  MESSAGE(" [x] Fetching new development firmware from Shadow's internal repository and headers.")
ELSE(internal_firmware)
  MESSAGE(" [x] Fetching latest RELEASED firmware and headers.")
ENDIF(internal_firmware)

IF(internal_firmware)
  EXECUTE_PROCESS(COMMAND "${PROJECT_SOURCE_DIR}/scripts/update_header.sh")
ELSE(internal_firmware)
  #EXECUTE_PROCESS(COMMAND "${PROJECT_SOURCE_DIR}/scripts/update_released_header.sh")
  EXECUTE_PROCESS(COMMAND "${PROJECT_SOURCE_DIR}/scripts/update_provisional_headers.sh")
ENDIF(internal_firmware)
