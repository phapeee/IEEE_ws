cmake_minimum_required(VERSION 3.0.2)
project(mecanum_ik)

## Use C++14 for compatibility with the I2CReader code
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  message_generation
)

add_service_files(
  FILES
  adjustPositionSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## catkin package configuration
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mecanum_ik i2c_reader
   CATKIN_DEPENDS geometry_msgs message_runtime roscpp
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build the I2CReader library
add_library(i2c_reader
  include/${PROJECT_NAME}/i2c_reader.cpp
)

## Add the main executable
add_executable(mecanum_ik_node src/mecanum_ik.cpp)

## Link the executable to the necessary libraries
target_link_libraries(mecanum_ik_node
  i2c_reader
  ${catkin_LIBRARIES}
  -lwiringPi
)

## Add dependencies for message generation
add_dependencies(mecanum_ik_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install targets
install(TARGETS mecanum_ik_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS i2c_reader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

