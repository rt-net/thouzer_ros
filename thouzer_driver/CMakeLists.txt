cmake_minimum_required(VERSION 3.0.2)
project(thouzer_driver)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/cmd_vel.py
  scripts/commander.py
  scripts/odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  find_package(rostest REQUIRED)
  add_rostest(test/commander_node.test)
endif()