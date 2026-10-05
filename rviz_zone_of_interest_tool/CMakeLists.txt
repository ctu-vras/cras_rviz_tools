# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(rviz_zone_of_interest_tool)

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

add_library(${PROJECT_NAME}_plugin SHARED
  src/rviz_zone_of_interest.cpp
  include/rviz_zone_of_interest_tool/rviz_zone_of_interest_tool.hpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  PRIVATE
    pluginlib::pluginlib
    polygon_msgs::polygon_msgs
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
)
target_include_directories(${PROJECT_NAME}_plugin PRIVATE include)

install(TARGETS ${PROJECT_NAME}_plugin EXPORT ${PROJECT_NAME}_targets
  DESTINATION lib
)
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
    polygon_msgs
    rviz_common
    rviz_rendering
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()
endif()

ament_package()
