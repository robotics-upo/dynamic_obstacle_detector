cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_detector)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_geometry
  roscpp
  sensor_msgs
  tf2_ros
  message_generation
)


 add_message_files(
   FILES
   DynamicObstacle.msg
   DynamicObstacleStamped.msg
   DynamicObstacles.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_obstacle_detector
  CATKIN_DEPENDS geometry_msgs laser_geometry roscpp sensor_msgs tf2_ros message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/dynamic_obstacle_detector.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
