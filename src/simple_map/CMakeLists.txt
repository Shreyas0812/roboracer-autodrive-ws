cmake_minimum_required(VERSION 3.8)
project(simple_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(simple_map_node src/simple_map_node.cpp)

ament_target_dependencies(simple_map_node 
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros
)

# Install Cpp executables
install(TARGETS
  simple_map_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/simple_map_node.py
  scripts/waypoint_logger_node.py
  scripts/visualise_node.py
  scripts/waypoint_saver_service.py
  DESTINATION lib/${PROJECT_NAME}
)

#Launch 
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Config Files
install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config
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
