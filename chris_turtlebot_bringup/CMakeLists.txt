cmake_minimum_required(VERSION 2.8.3)
project(chris_turtlebot_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  simple_message_to_tf
  robot_state_publisher
  tf2_ros
  chris_turtlebot_model
  stamped_cmd_vel_mux
  stamped_velocity_smoother
  laptop_battery_monitor
  urg_node
  laser_filters
)


catkin_package( )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)
