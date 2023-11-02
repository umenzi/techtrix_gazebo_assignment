cmake_minimum_required(VERSION 3.1.3)
cmake_policy(VERSION 3.1.3)
project(techtrix_grabber_contact_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(techtrix_grabber_contact_plugin_name GrabberContactPlugin)
add_library(${techtrix_grabber_contact_plugin_name} SHARED src/ContactPlugin.cc)
target_link_libraries(${techtrix_grabber_contact_plugin_name} ${GAZEBO_LIBRARIES})

install(TARGETS ${conveyor_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)