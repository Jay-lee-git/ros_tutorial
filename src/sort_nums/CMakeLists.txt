cmake_minimum_required(VERSION 3.5)
project(sort_nums)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)


add_executable(sort_service src/sort_service.cpp)
ament_target_dependencies(sort_service rclcpp tutorial_interfaces)

add_executable(sort_client src/sort_client.cpp)
ament_target_dependencies(sort_client rclcpp tutorial_interfaces)


install(TARGETS
  sort_service
  sort_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
