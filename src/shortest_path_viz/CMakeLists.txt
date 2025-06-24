cmake_minimum_required(VERSION 3.8)
project(shortest_path_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(src)

# Add executable
add_executable(shortest_path_visualizer 
  src/shortest_path_visualizer.cpp
  src/ShortestPathLib.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(shortest_path_visualizer
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  shortest_path_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  ../../models/
  DESTINATION share/${PROJECT_NAME}/models/
  FILES_MATCHING PATTERN "*.off"
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
