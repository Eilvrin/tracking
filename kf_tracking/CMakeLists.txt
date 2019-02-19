cmake_minimum_required(VERSION 3.0)
project(kf_tracking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  simple_data_publisher
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Tracklet.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp simple_data_publisher geometry_msgs
   DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME} ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})


add_executable(${PROJECT_NAME}_node src/kf_tracking_node.cpp src/kf_tracking.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
