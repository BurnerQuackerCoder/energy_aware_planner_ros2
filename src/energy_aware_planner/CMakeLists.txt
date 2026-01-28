cmake_minimum_required(VERSION 3.8)
project(energy_aware_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

# Create the Library
add_library(${PROJECT_NAME} SHARED
  src/energy_aware_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  nav2_core
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  visualization_msgs
  geometry_msgs
  pluginlib
  tf2_ros
)

# Export the plugin
pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/battery_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()