cmake_minimum_required(VERSION 2.8.3)
project(chris_turtlebot_model)

find_package(gazebo REQUIRED)


find_package(catkin REQUIRED COMPONENTS gazebo_ros
                                        gazebo_plugins
                                        geometry_msgs
                                        kobuki_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf2_ros)

catkin_package(INCLUDE_DIRS include
               LIBRARIES chris_gazebo_ros_kobuki
               CATKIN_DEPENDS gazebo_ros
                              turtlebot_description
                              kobuki_description
                              gazebo_plugins
                              geometry_msgs
                              kobuki_msgs
                              nav_msgs
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf2_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

add_library(chris_gazebo_ros_kobuki src/gazebo_ros_kobuki.cpp
                                    src/gazebo_ros_kobuki_updates.cpp
                                    src/gazebo_ros_kobuki_loads.cpp)

add_dependencies(chris_gazebo_ros_kobuki ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chris_gazebo_ros_kobuki
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS chris_gazebo_ros_kobuki
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)
