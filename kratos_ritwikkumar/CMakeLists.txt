cmake_minimum_required(VERSION 3.8)
project(kratos_ritwikkumar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverStatus.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)
# Install Python module (so you can import it if needed)
install(
  DIRECTORY kratos_ritwikkumar
  DESTINATION lib/python3.10/site-packages
)

# Install your executable Python script
install(
  PROGRAMS
  kratos_ritwikkumar/clock_publisher.py
  kratos_ritwikkumar/rover_status_publisher.py
  kratos_ritwikkumar/hello_publisher_q1.py
  kratos_ritwikkumar/hello_subscriber_q1.py
  kratos_ritwikkumar/s1_controller_q2.py
  kratos_ritwikkumar/s2_controller_q2.py
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
ament_export_dependencies(rosidl_default_runtime)

ament_package()
