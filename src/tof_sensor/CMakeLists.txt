cmake_minimum_required(VERSION 3.0.2)
project(tof_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Tof.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tof_sensor_node src/tof_sensor_node.cpp)
add_executable(cova_calc_node src/covariance_calculation.cpp)
add_dependencies(tof_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tof_sensor_node ${catkin_LIBRARIES})
target_link_libraries(cova_calc_node ${catkin_LIBRARIES})
