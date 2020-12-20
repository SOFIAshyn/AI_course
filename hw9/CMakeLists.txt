cmake_minimum_required(VERSION 2.8.3)
project(dist)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# add_executable(talker src/talker.cpp)
add_executable(dist src/dist.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# target_link_libraries(
#   talker  ${catkin_LIBRARIES}
# )

target_link_libraries(
  dist  ${catkin_LIBRARIES}
)