cmake_minimum_required(VERSION 3.0.2)
project(ackermann_odom_tf)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)


add_message_files(
  FILES
  Odom.msg
)

add_service_files(
  FILES
  ResetOdom.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(odom_node src/odom_node.cpp)

target_link_libraries(odom_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  odom_node ackermann_odom_tf_generate_messages_cpp
)

add_executable(reset_odom_client src/reset_odom_client.cpp)

target_link_libraries(reset_odom_client ${catkin_LIBRARIES})

add_dependencies(
  reset_odom_client ${catkin_EXPORTED_TARGETS}
  odom_node ${catkin_EXPORTED_TARGETS}
)
