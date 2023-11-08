cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
)

set(srv_files
  "srv/AddFourInts.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 상기 매크로는 cmake의 타겟을 ${PROEJCT_Name}을 통해서 찾으며 c++을 지원한다는 뜻임
# rosidl_get_typesupport_target: 생성된 메세지/서비스 타입 지원 라이브러리를 c++실행 파일과 연결시켜줌
# cpp_typesupport_target: CMake의 타겟 이름 포함
  # 타겟이란 publish_address_book과 같은것을 말함
# rosidl_typesupport_cpp: C++ 타입 지원을 위한 식별자

target_link_libraries(publish_address_book "${cpp_typesupport_target}")

install(TARGETS
    publish_address_book
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
