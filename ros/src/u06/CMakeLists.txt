cmake_minimum_required(VERSION 2.8.3)
project(u06)

find_package(catkin REQUIRED roscpp std_msgs tf message_generation)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(u06_tf_listener src/arm2r_tf_listener.cpp)
target_link_libraries(u06_tf_listener ${catkin_LIBRARIES})
