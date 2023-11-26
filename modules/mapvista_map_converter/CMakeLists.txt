cmake_minimum_required(VERSION 3.8)
project(mapvista_map_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mapvista_map_msgs REQUIRED)

set(map_converter_executable map_converter)
set(map_converter_library ${map_converter_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  octomap
  octomap_msgs
  octomap_ros
  pcl_conversions
  sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${map_converter_executable}
  src/map_converter_node.cpp
  src/map_converter_component.cpp
)
add_library(${map_converter_library} SHARED
  src/map_converter_component.cpp
)
ament_target_dependencies(${map_converter_executable}
  ${dependencies}
)
ament_target_dependencies(${map_converter_library}
  ${dependencies}
)
target_link_libraries(${map_converter_executable}
  ${PCL_LIBRARIES}
)
target_link_libraries(${map_converter_library}
  ${PCL_LIBRARIES}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rclcpp_components_register_nodes(${map_converter_library}
  "mapvista_map_converter::MapConverter"
)

install(TARGETS ${map_converter_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${map_converter_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format(
    CONFIG_FILE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format>
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${map_converter_library})
ament_export_dependencies(${dependencies})
ament_package()
