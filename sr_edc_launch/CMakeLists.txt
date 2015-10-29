# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_edc_launch)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(FILES sr_edc.launch sr_edc.machine sr_edc_bimanual.launch load_hand_parameters.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY mappings
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
