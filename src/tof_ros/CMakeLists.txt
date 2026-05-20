cmake_minimum_required(VERSION 3.0.2)
project(tof_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs tf2_ros pcl_ros pcl_conversions
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(tof_receiver src/tof_receiver.cpp)
add_dependencies(tof_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tof_receiver ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tof_localizer src/tof_localizer.cpp)
add_dependencies(tof_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tof_localizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS tof_receiver tof_localizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
