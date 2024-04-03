cmake_minimum_required(VERSION 3.5)
project(microstrain_inertial_localization)

# find dependencies
find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()