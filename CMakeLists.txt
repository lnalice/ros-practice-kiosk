cmake_minimum_required(VERSION 3.0.2)
project(my_kiosk)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
## catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    Order.msg

)
## Generate actions in the 'action' folder
add_action_files(
	FILES
	Kiosk.action

)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_messages(
  DEPENDENCIES
  actionlib_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_kiosk
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########


#############
## Install ##
#############

install(DIRECTORY examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
