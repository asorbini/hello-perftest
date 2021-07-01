cmake_minimum_required(VERSION 3.5)
project(hello_perftest)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(rclcpp_components)
find_package(${PROJECT_NAME}_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/latency/latency.cpp
  include/hello_perftest/latency.hpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  ${PROJECT_NAME}_interfaces
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hello_perftest::LatencySubscriber"
  EXECUTABLE latency_sub
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hello_perftest::LatencyPublisher"
  EXECUTABLE latency_pub
)

add_executable(latency_sub_main
  src/latency/latency_subscriber_main.cpp
)
target_link_libraries(latency_sub_main ${PROJECT_NAME})
install(TARGETS latency_sub_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(latency_pub_main
  src/latency/latency_publisher_main.cpp
)
target_link_libraries(latency_pub_main ${PROJECT_NAME})
install(TARGETS latency_pub_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(latency_single
  src/latency/latency_single_process.cpp
)
target_link_libraries(latency_single ${PROJECT_NAME})
install(TARGETS latency_single
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
