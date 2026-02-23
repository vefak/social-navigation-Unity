cmake_minimum_required(VERSION 3.8)
project(floor_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-w)  # Disables all warnings

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(pybind11 REQUIRED)
find_package(unitycustommsg REQUIRED)
find_package(nav2_msgs REQUIRED)


# Include Python and pybind11 headers
include_directories(${Python3_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS})

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
# Define the executable
add_executable(local_planner src/local_planner.cpp)

# Set ament dependencies for each target
ament_target_dependencies(local_planner rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros OpenCV nav_msgs unitycustommsg nav2_msgs)

set(SUBMODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../floor-segmentation)
add_subdirectory(${SUBMODULE_PATH} ${CMAKE_BINARY_DIR}/floor-segmentation-build)

# Link libraries to the executable
target_link_libraries(local_planner geometry graph_util grid planning database social_force util potential save nlohmann ${OpenCV_LIBS})
target_include_directories(local_planner PRIVATE ${SUBMODULE_PATH})
set(CMAKE_BUILD_TYPE Debug)


# Install targets and directories
install(TARGETS local_planner database geometry graph_util grid nlohmann planning save social_force util potential
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



