cmake_minimum_required(VERSION 3.8)
project(mapvista_moving_least_squares)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)

set(moving_least_squares_executable moving_least_squares)
set(moving_least_squares_library ${moving_least_squares_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  pcl_conversions
  sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${moving_least_squares_executable}
  src/moving_least_squares_node.cpp
  src/moving_least_squares_component.cpp
)
add_library(${moving_least_squares_library} SHARED
  src/moving_least_squares_component.cpp
)
ament_target_dependencies(${moving_least_squares_executable}
  ${dependencies}
)
ament_target_dependencies(${moving_least_squares_library}
  ${dependencies}
)
target_link_libraries(${moving_least_squares_executable}
  ${PCL_LIBRARIES})
target_link_libraries(${moving_least_squares_library}
  ${PCL_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rclcpp_components_register_nodes(${moving_least_squares_library}
  "mapvista_moving_least_squares::MovingLeastSquares"
)

install(TARGETS ${moving_least_squares_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${moving_least_squares_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format(
    CONFIG_FILE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../.clang-format>
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${moving_least_squares_library})
ament_export_dependencies(${dependencies})
ament_package()
