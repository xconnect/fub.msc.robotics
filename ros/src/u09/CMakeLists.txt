cmake_minimum_required(VERSION 2.8.3)
project(u09)

find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(astarPose src/Astar.cpp src/Astar.h src/Node.h src/Node.cpp src/Pose.h src/Pose.cpp)
target_link_libraries(astarPose ${catkin_LIBRARIES} ${OpenCV_LIBS})
