cmake_minimum_required(VERSION 3.8)
project(learning_tf2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(message_filters REQUIRED)

add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
  static_turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
  tf2
)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
  turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
  tf2
  turtlesim
)

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(
  turtle_tf2_listener
  geometry_msgs
  rclcpp
  tf2_ros
  tf2
  turtlesim
)

add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  fixed_frame_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
  tf2
  turtlesim
)

add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  dynamic_frame_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
)

add_executable(quaternion_example src/quaternion_example.cpp)
ament_target_dependencies(
  quaternion_example
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(turtle_tf2_message_broadcaster src/turtle_tf2_message_broadcaster.cpp)
ament_target_dependencies(
  turtle_tf2_message_broadcaster
  rclcpp
  geometry_msgs
  turtlesim
  tf2_ros
  tf2
  )


add_executable(turtle_tf2_message_filter src/turtle_tf2_message_filter.cpp)
ament_target_dependencies(
  turtle_tf2_message_filter
  geometry_msgs
  message_filters
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  )

install(TARGETS
  static_turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  turtle_tf2_listener
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  fixed_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})  
  
install(TARGETS
  dynamic_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})  
  
install(TARGETS
  quaternion_example
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  turtle_tf2_message_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  turtle_tf2_message_filter
  DESTINATION lib/${PROJECT_NAME})

# install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
