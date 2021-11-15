cmake_minimum_required(VERSION 3.5.1)
project(dual_arm_api)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin
  REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  dynamic_reconfigure
  roscpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  image_transport
  message_generation
  OPTIONAL_COMPONENTS
  roport
)

if (${roport_FOUND})
  add_definitions(-DUSE_ROPORT)
  MESSAGE(STATUS "FOUND roport")
else (${roport_FOUND})
  MESSAGE(STATUS "NOT FOUND roport")
endif (${roport_FOUND})

find_package(OpenCV REQUIRED)

# Set BOOST_LIBRARYDIR accordingly in case that PCL could not find Boost
set(WEBOTS_DIR /usr/local/webots)

add_service_files(
  FILES
  MoveAndCapture.srv
  MoveJointGroup.srv
)

#add_action_files(
#  DIRECTORY action
#)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Link with the Webots controller library.
link_directories(${WEBOTS_DIR}/lib/controller)
set(
  WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(
  ${WEBOTS_DIR}/include/controller/c
  ${WEBOTS_DIR}/include/controller/cpp
)


# Declare a C++ library
add_library(${PROJECT_NAME}_lib
  lib/RobotNode.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${WEBOTS_LIB}
)

add_executable(dual_arm_api dual_arm_api.cpp)
add_dependencies(dual_arm_api ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_arm_api
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${WEBOTS_LIB}
  ${PROJECT_NAME}_lib
)

# Get the parent folder of the build folder (i.e., ROS workspace)
get_filename_component(ROS_WORKSPACE ${CMAKE_BINARY_DIR} DIRECTORY)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${ROS_WORKSPACE}/devel/lib/${PROJECT_NAME}/${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/nachi_ros/${PROJECT_NAME}
)

add_custom_command(TARGET ${PROJECT_NAME}_lib POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${ROS_WORKSPACE}/devel/lib/lib${PROJECT_NAME}_lib.so ${CMAKE_SOURCE_DIR}/nachi_ros/${PROJECT_NAME}
)

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
#install(TARGETS ${PROJECT_NAME}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

