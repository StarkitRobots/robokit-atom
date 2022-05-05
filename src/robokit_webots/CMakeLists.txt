cmake_minimum_required(VERSION 2.8.3)
project(robokit_webots)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation tf moveit_ros_planning_interface)


add_message_files(
  FILES
  servo_command.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES webots_ros
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime tf
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/servo_control.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
catkin_install_python(PROGRAMS scripts/example_servo_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/start_sensors.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/executor.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
