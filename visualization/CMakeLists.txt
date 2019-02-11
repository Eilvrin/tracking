cmake_minimum_required(VERSION 2.8.3)
project(visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  simple_data_publisher
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp visualization_msgs simple_data_publisher
   # DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(visualization_node src/visualization_node.cpp src/visualization.cpp)
target_link_libraries(visualization_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
