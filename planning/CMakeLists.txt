cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
)


catkin_install_python(PROGRAMS
        src/trajectory.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
