cmake_minimum_required(VERSION 3.5)
project(unitycustommsg)

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TwistTransformUnity.msg"
  "msg/Point2D.msg"
  "msg/Point2DArray.msg"
  "msg/SocialMetrics.msg"
  "msg/Human.msg"
  "msg/HumanArray.msg"
  "msg/HumanGroup.msg"
  "msg/HumanGroupArray.msg"
  "msg/HumanInvasion.msg"
  "msg/HumanInvasionArray.msg"

  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()





