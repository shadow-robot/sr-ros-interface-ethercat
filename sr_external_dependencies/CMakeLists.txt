# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_external_dependencies)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED )


SET(internal_firmware $ENV{INTERNAL_FIRMWARE})
IF(internal_firmware)
  MESSAGE(" [x] Fetching new development firmware from Shadow's internal repository and headers.")
ELSE(internal_firmware)
  MESSAGE(" [x] Fetching latest RELEASED firmware and headers.")
ENDIF(internal_firmware)

IF(internal_firmware)
  EXECUTE_PROCESS(COMMAND "${PROJECT_SOURCE_DIR}/scripts/update_header.sh")
ELSE(internal_firmware)
  EXECUTE_PROCESS(COMMAND "${PROJECT_SOURCE_DIR}/scripts/update_released_header.sh")
ENDIF(internal_firmware)


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    INCLUDE_DIRS include
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY released/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/released)

install(FILES tested_version.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


