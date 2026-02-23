cmake_minimum_required(VERSION 3.8)
project(stanley_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)  
find_package(tf2 REQUIRED)  
find_package(unitycustommsg REQUIRED)

# Suppress "unused but set variable" warnings
add_compile_options(-Wno-unused-but-set-variable)

# Include directories
include_directories(include/stanley_controller ${EIGEN3_INCLUDE_DIR})

# Define the executables

add_executable(stanley src/stanley_controller.cpp)


# Set ament dependencies for each target
ament_target_dependencies(stanley rclcpp std_msgs geometry_msgs tf2 unitycustommsg) 

# Link Eigen to main_node (not needed for talker and listener)
target_link_libraries(stanley Eigen3::Eigen)

# Install the targets
install(TARGETS
  stanley
  DESTINATION lib/${PROJECT_NAME})

# ament package
ament_package()
