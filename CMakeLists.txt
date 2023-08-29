cmake_minimum_required(VERSION 3.0.2)
project(points_global_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  map_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    map_msgs
    nav_core
    nav_msgs
    navfn
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/customplanner.cpp
  src/create_points.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(create_points_node src/create_points_node.cpp)
add_dependencies(create_points_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_points_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS create_points_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

