cmake_minimum_required(VERSION 2.8.3)
project(u08)

find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(astar src/Astar.cpp src/Astar.h src/Node.h src/Node.cpp)
target_link_libraries(astar ${catkin_LIBRARIES} ${OpenCV_LIBS})
