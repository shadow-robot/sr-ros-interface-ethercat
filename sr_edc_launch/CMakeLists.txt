# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_edc_launch)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS sr_ethercat_hand_config roscpp diagnostic_aggregator diagnostic_updater diagnostic_msgs self_test urdf xacro convex_decomposition ivcon pr2_etherCAT pr2_bringup robot_mechanism_controllers sr_description
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


install(FILES sr_edc.launch sr_edc.machine
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


