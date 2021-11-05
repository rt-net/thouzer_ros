cmake_minimum_required(VERSION 3.0.2)
project(thouzer_msgs)

set(MESSAGE_DEPENDS
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation ${MESSAGE_DEPENDS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  App.msg
  Battery.msg
  Commander.msg
  Odom.msg
  StringStamped.msg
  Vel.msg
)

## Generate messages in the 'srv' folder
##add_service_files(
##  FILES
##  Motor.srv
##)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MESSAGE_DEPENDS}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  message_runtime
  ${MESSAGE_DEPENDS}
)
