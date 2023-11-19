cmake_minimum_required(VERSION 3.8)
project(panel_detector_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/panel_detector.cpp
)
ament_target_dependencies(${PROJECT_NAME} OpenCV)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::PanelDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_exec)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
ament_auto_package()
