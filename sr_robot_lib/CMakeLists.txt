# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_robot_lib)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS sr_utilities sr_hardware_interface sr_external_dependencies sr_robot_msgs sr_ethercat_hand_config std_srvs diagnostic_updater realtime_tools pr2_mechanism_msgs sr_self_test roscpp rospy)

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

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories( ${ImageMagick_INCLUDE_DIRS})

#if you compile with "DEBUG=1 make", some debug data are going to be published
SET(debug $ENV{DEBUG})
MESSAGE(" ----- Shadow Robot EtherCAT driver configuration:")
IF(DEFINED debug)
  SET( CMAKE_CXX_FLAGS "-D DEBUG_PUBLISHER")
  MESSAGE(" [x] Debug publisher")
ELSE(DEFINED debug)
  MESSAGE(" [ ] Debug publisher")
ENDIF(DEFINED debug)
MESSAGE(" ----- ")

add_library(sr_hand_lib src/UBI0.cpp src/biotac.cpp src/generic_tactiles.cpp src/generic_updater.cpp src/motor_data_checker.cpp src/motor_updater.cpp src/muscle_updater.cpp src/sensor_updater.cpp src/shadow_PSTs.cpp src/sr_motor_hand_lib.cpp src/sr_robot_lib.cpp src/sr_motor_robot_lib.cpp src/sr_muscle_hand_lib.cpp src/sr_muscle_robot_lib.cpp)

target_link_libraries(sr_hand_lib sr_self_test)
###############
#     TESTS
SET(jenkins $ENV{JENKINS})

IF(DEFINED jenkins)
  add_library(gcov STATIC IMPORTED)
  set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
ENDIF(DEFINED jenkins)

add_executable(test/test_robot_lib test/test_robot_lib.cpp)
target_link_libraries(test/test_robot_lib sr_hand_lib)
IF(DEFINED jenkins)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
#   rosbuild_add_compile_flags(test/test_robot_lib -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_robot_lib gcov)
ENDIF(DEFINED jenkins)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(test/test_robot_lib)
add_rostest(launch/sr_hand_lib.test)

#add_executable(test/motor_updater_test test/motor_updater_test.cpp src/motor_updater.cpp src/generic_updater.cpp)
add_executable(test/motor_updater_test test/motor_updater_test.cpp)
target_link_libraries(test/motor_updater_test sr_hand_lib)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(test/motor_updater_test)
IF(DEFINED jenkins)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
#   rosbuild_add_compile_flags(test/motor_updater_test -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/motor_updater_test gcov)
ENDIF(DEFINED jenkins)
add_rostest(test/motor_updater.test)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS sr_utilities sr_hardware_interface sr_external_dependencies sr_robot_msgs sr_ethercat_hand_config std_srvs diagnostic_updater realtime_tools pr2_mechanism_msgs sr_self_test roscpp rospy
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)