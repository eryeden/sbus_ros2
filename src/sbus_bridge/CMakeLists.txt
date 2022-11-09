cmake_minimum_required(VERSION 3.8)
project(sbus_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sbus_interface REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(sbus_bridge_node
        src/sbus_bridge_node.cpp
        src/sbus_serial_driver.cpp
        )
ament_target_dependencies(sbus_bridge_node rclcpp std_msgs sbus_interface)

add_executable(sbus_calib
        src/sbus_calib.cpp
        src/sbus_serial_driver.cpp
        )
ament_target_dependencies(sbus_calib rclcpp std_msgs sbus_interface)


install(TARGETS
        sbus_bridge_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        sbus_calib
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME})

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
