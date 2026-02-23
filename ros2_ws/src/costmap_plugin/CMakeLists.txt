cmake_minimum_required(VERSION 3.8)
project(costmap_plugin)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(unitycustommsg REQUIRED)
find_package(tf2 REQUIRED)  


# Suppress "unused but set variable" warnings
add_compile_options(-Wno-unused-but-set-variable)
set(CMAKE_SKIP_RPATH TRUE)
# Plugin source
add_library(dynamic_human_layer SHARED
  src/dynamic_human_layer.cpp
)

ament_target_dependencies(dynamic_human_layer
  nav2_costmap_2d
  rclcpp
  pluginlib
  unitycustommsg
  geometry_msgs
  tf2
  
)

target_include_directories(dynamic_human_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Register the plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

# Install pluginlib.xml and headers
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

# Install the plugin library
install(TARGETS dynamic_human_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
