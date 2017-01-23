cmake_minimum_required(VERSION 2.8.3)
project(u10)

find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(potentialField src/Astar.cpp src/Astar.h src/Node.h src/Node.cpp)
target_link_libraries(potentialField ${catkin_LIBRARIES} ${OpenCV_LIBS})
