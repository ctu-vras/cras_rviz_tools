# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(cras_rviz_vision_msgs)

set(CMAKE_AUTOMOC ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(${PROJECT_NAME}_plugin SHARED
  src/detection2d_array_display.cpp
  src/detection_buffer.cpp
  src/detection_store.cpp
  src/image_cache.cpp
  src/image_topic_property.cpp
  src/layout_planner.cpp
  include/cras_rviz_vision_msgs/detection2d_array_display.hpp
  include/cras_rviz_vision_msgs/detection_buffer.hpp
  include/cras_rviz_vision_msgs/detection_store.hpp
  include/cras_rviz_vision_msgs/image_cache.hpp
  include/cras_rviz_vision_msgs/image_topic_property.hpp
  include/cras_rviz_vision_msgs/layout_planner.hpp
)
target_include_directories(${PROJECT_NAME}_plugin PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_plugin
  PRIVATE
    cras_cpp_common::cras_image_transport
    cv_bridge::cv_bridge
    image_transport::image_transport
    pluginlib::pluginlib
    rclcpp::rclcpp
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${vision_msgs_TARGETS}
    ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_plugin EXPORT ${PROJECT_NAME}_targets
  DESTINATION lib
)
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
  cras_cpp_common
  cv_bridge
  geometry_msgs
  image_transport
  pluginlib
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs
  vision_msgs
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS demo/fake_detector.py DESTINATION share/${PROJECT_NAME}/demo)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  ament_add_gtest(${PROJECT_NAME}_test_layout_planner test/test_layout_planner.cpp src/layout_planner.cpp)
  if(TARGET ${PROJECT_NAME}_test_layout_planner)
    target_include_directories(${PROJECT_NAME}_test_layout_planner PRIVATE include)
  endif()

  ament_add_gtest(${PROJECT_NAME}_test_detection_store test/test_detection_store.cpp src/detection_store.cpp)
  if(TARGET ${PROJECT_NAME}_test_detection_store)
    target_include_directories(${PROJECT_NAME}_test_detection_store PRIVATE include)
  endif()

  ament_add_gtest(${PROJECT_NAME}_test_detection_buffer test/test_detection_buffer.cpp src/detection_buffer.cpp)
  if(TARGET ${PROJECT_NAME}_test_detection_buffer)
    target_include_directories(${PROJECT_NAME}_test_detection_buffer PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_test_detection_buffer
      cv_bridge::cv_bridge
      rclcpp::rclcpp
      rviz_rendering::rviz_rendering
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${OpenCV_LIBRARIES}
    )
  endif()

  ament_add_gtest(${PROJECT_NAME}_test_image_cache test/test_image_cache.cpp src/image_cache.cpp)
  if(TARGET ${PROJECT_NAME}_test_image_cache)
    target_include_directories(${PROJECT_NAME}_test_image_cache PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_test_image_cache
      cv_bridge::cv_bridge
      rclcpp::rclcpp
      rviz_rendering::rviz_rendering
      ${sensor_msgs_TARGETS}
      ${OpenCV_LIBRARIES}
    )
  endif()
endif()

ament_package()
