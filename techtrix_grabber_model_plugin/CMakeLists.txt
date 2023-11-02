cmake_minimum_required(VERSION 3.1.3)
cmake_policy(VERSION 3.1.3)
project(techtrix_grabber_model_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/grabber_model_plugin.cc)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


foreach (dir launch worlds)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
