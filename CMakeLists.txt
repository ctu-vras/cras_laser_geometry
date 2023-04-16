cmake_minimum_required(VERSION 3.10.2)
project(cras_laser_geometry)

find_package(catkin REQUIRED
  COMPONENTS
    cras_cpp_common
    laser_geometry
    nodelet
    roscpp
    sensor_msgs
    tf2_ros
)
find_package(OpenCV REQUIRED COMPONENTS core)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugins src/${PROJECT_NAME}/scan_to_point_cloud2.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_plugins
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

