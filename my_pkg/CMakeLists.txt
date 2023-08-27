cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)
find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    sensor_msgs
    detection_msgs
)
catkin_package(CATKIN_DEPENDS
    rospy geometry_msgs sensor_msgs detection_msgs
)
catkin_install_python(PROGRAMS
    scripts/my_prog.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
