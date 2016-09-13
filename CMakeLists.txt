cmake_minimum_required(VERSION 2.8.3)
project(sonar_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonar_to_laserscan
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sonar_to_laserscan src/sonar_to_laserscan_nodelet.cpp)
target_link_libraries(sonar_to_laserscan ${catkin_LIBRARIES})

add_executable(sonar_to_laserscan_node src/sonar_to_laserscan_node.cpp)
target_link_libraries(sonar_to_laserscan_node sonar_to_laserscan ${catkin_LIBRARIES})

install(TARGETS sonar_to_laserscan sonar_to_laserscan_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
