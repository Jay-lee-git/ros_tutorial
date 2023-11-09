cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp)


# rosidl_get_typesupport_target로 이 라이브러리를 설정하며, 타겟을 "cpp_typesupport_target" 라는 변수로 설정
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(talker "${cpp_typesupport_target}")
target_link_libraries(listener "${cpp_typesupport_target}")

# rosidl_default_runtime 패키지에 의존성을 띈다고 명시적으로 선언
ament_export_dependencies(rosidl_default_runtime)


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
