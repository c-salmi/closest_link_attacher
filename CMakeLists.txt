cmake_minimum_required(VERSION 3.0.2)
project(closest_link_attacher)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

add_service_files(
  FILES
  Attach.srv
  Detach.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
# include
  ${catkin_INCLUDE_DIRS},
  ${GAZEBO_INCLUDE_DIRS}
)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(closest_link_attacher SHARED src/closest_link_attacher.cpp)
target_link_libraries(closest_link_attacher ${GAZEBO_LIBRARIES})
add_dependencies(closest_link_attacher ${closest_link_attacher_EXPORTED_TARGETS})

