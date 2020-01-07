cmake_minimum_required(VERSION 2.8.3)

project(vfly)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  mav_msgs
  message_generation
  adrc_control
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  vfly_pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  # LIBRARIES pid 
  CATKIN_DEPENDS roscpp  sensor_msgs  message_runtime geometry_msgs mav_msgs
  DEPENDS Eigen3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(omni_pid_controller_node src/omni_pid_controller_node.cpp)
target_link_libraries(omni_pid_controller_node pid ${catkin_LIBRARIES})
add_dependencies(omni_pid_controller_node ${vfly}_generate_messages_cpp)

add_executable(omni_adrc_controller_node src/omni_adrc_controller_node.cpp)
target_link_libraries(omni_adrc_controller_node  adrc ${catkin_LIBRARIES})
add_dependencies(omni_adrc_controller_node ${vfly}_generate_messages_cpp)