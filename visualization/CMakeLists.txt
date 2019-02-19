cmake_minimum_required(VERSION 2.8.3)
project(visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kf_tracking
  roscpp
  rviz
  simple_data_publisher
  visualization_msgs
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp visualization_msgs simple_data_publisher kf_tracking
   DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(tracking_display include/visualization/tracking_display.h src/tracking_display.cpp include/visualization/tracklet_visual.h src/tracklet_visual.cpp)
target_link_libraries(tracking_display ${catkin_LIBRARIES} ${QT_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(visualization_node src/visualization_node.cpp
                                  src/visualization.cpp
                                  )
target_link_libraries(visualization_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node tracking_display
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

