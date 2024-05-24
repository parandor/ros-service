cmake_minimum_required(VERSION 3.8)
project(cpp_parameters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)  # Add this line
find_package(std_msgs REQUIRED)

# Find and link against cpprestsdk library
find_package(cpprestsdk REQUIRED)
add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp rclcpp_components std_msgs)

find_package(OpenSSL REQUIRED)
target_link_libraries(minimal_param_node OpenSSL::SSL OpenSSL::Crypto)

include_directories(
  ${std_msgs_INCLUDE_DIRS}
)

if(cpprestsdk_FOUND)
  # Add cpprestsdk include directories
  include_directories(${cpprestsdk_INCLUDE_DIRS})

  # Add cpprestsdk libraries
  target_link_libraries(minimal_param_node ${cpprestsdk_LIBRARIES} cpprestsdk::cpprest)
else()
  message(FATAL_ERROR "cpprestsdk not found. Install libcpprest-dev package.")
endif()

install(TARGETS
    minimal_param_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
