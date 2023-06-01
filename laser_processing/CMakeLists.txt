cmake_minimum_required(VERSION 3.0.2)
project(laser_processing)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  laser_line_extraction
)

add_message_files(
  FILES
  Lanes.msg
  ControlDebug.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_processing_ros
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp sensor_msgs
)

add_library(laser_processing_ros src/laser_processing_ros.cpp)
target_link_libraries(laser_processing_ros ${catkin_LIBRARIES})
add_dependencies(laser_processing_ros laser_processing_generate_messages_cpp)

add_executable(laser_processing_node src/laser_processing_node.cpp)
target_link_libraries(laser_processing_node laser_processing_ros ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})

# catkin_add_gtest(${PROJECT_NAME}-test test/test_laser_laser_processing.cpp)

install(TARGETS laser_processing_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS laser_processing_ros ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
