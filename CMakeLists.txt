cmake_minimum_required(VERSION 3.0)
project(rgb2ii)
set(ROS_VERSION $ENV{ROS_DISTRO})

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rospy
        sensor_msgs
        std_msgs
        cv_bridge
        )

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge
)
catkin_python_setup()

catkin_install_python(
  PROGRAMS
    script/rgb2ii.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
         PATTERN ".svn" EXCLUDE
         )

