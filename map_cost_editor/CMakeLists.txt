cmake_minimum_required(VERSION 3.0.2)
project(map_cost_editor)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(add_cost_node src/add_cost_node.cpp)

target_link_libraries(add_cost_node
  ${catkin_LIBRARIES}
)

add_executable(obstacle_cost_node src/update_cost.cpp)

target_link_libraries(obstacle_cost_node ${catkin_LIBRARIES})

add_dependencies(obstacle_cost_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(set_obstacle_cost_node src/set_obstacle_cost_node.cpp)
target_link_libraries(set_obstacle_cost_node ${catkin_LIBRARIES})

add_dependencies(add_cost_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


