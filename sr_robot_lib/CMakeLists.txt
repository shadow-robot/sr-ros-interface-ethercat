# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.0.2)
project(sr_robot_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        sr_interpolation
        sr_utilities
        sr_hardware_interface
        sr_mechanism_model
        sr_external_dependencies
        sr_robot_msgs
        std_srvs
        cmake_modules
        diagnostic_updater
        realtime_tools
        controller_manager_msgs
        roscpp
        rostest
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(TinyXML2 REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#if you compile with "DEBUG=1 make", some debug data are going to be published
SET(debug $ENV{DEBUG})
MESSAGE(STATUS "Shadow Robot EtherCAT driver configuration:")
IF (DEFINED debug)
    SET(CMAKE_CXX_FLAGS "-D DEBUG_PUBLISHER")
    MESSAGE(STATUS " [x] Debug publisher")
ELSE(DEFINED debug)
    MESSAGE(STATUS " [ ] Debug publisher")
ENDIF (DEFINED debug)

catkin_python_setup()

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
        CATKIN_DEPENDS
        sr_interpolation
        sr_utilities
        sr_hardware_interface
        sr_mechanism_model
        sr_external_dependencies
        sr_robot_msgs
        std_srvs
        diagnostic_updater
        realtime_tools
        controller_manager_msgs
        roscpp
        rospy
        INCLUDE_DIRS include
        LIBRARIES sr_hand_lib
	DEPENDS TinyXML2
)

add_library(sr_hand_lib
        src/UBI0.cpp
        src/MST.cpp
        src/biotac.cpp
        src/generic_tactiles.cpp
        src/generic_updater.cpp
        src/motor_data_checker.cpp
        src/motor_updater.cpp
        src/muscle_updater.cpp
        src/sensor_updater.cpp
        src/shadow_PSTs.cpp
        src/sr_motor_hand_lib.cpp
        src/sr_robot_lib.cpp
        src/sr_motor_robot_lib.cpp
        src/sr_muscle_hand_lib.cpp
        src/sr_muscle_robot_lib.cpp
)

target_link_libraries(sr_hand_lib ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(sr_hand_lib ${catkin_EXPORTED_TARGETS}) 

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(
#  ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

###############
#     TESTS
###############
SET(jenkins $ENV{JENKINS})

IF (DEFINED jenkins)
    add_library(gcov STATIC IMPORTED)
    set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
ENDIF (DEFINED jenkins)

if(CATKIN_ENABLE_TESTING)
    if (COMMAND add_rostest_gtest)
        include_directories(${TinyXML2_INCLUDE_DIRS})
        add_rostest_gtest(test_robot_lib test/sr_hand_lib.test test/test_robot_lib.cpp)
        target_link_libraries(test_robot_lib sr_hand_lib gcov ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES} ${GTEST_LIBRARIES})

        add_rostest_gtest(motor_updater_test test/motor_updater.test test/motor_updater_test.cpp)
        target_link_libraries(motor_updater_test sr_hand_lib gcov ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    endif (COMMAND add_rostest_gtest)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS sr_hand_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        examples/compute_joint_0_target.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
