cmake_minimum_required(VERSION 3.8)
project(mini_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pipe -Wno-psabi)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(mini_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include Cpp "include" directory
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/spot.cpp
  src/${PROJECT_NAME}/teleop.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp mini_interfaces std_srvs std_msgs sensor_msgs geometry_msgs)

# Create Cpp executable
add_executable(spot_sm src/spot_sm.cpp)
add_executable(teleop_node src/teleop_node.cpp)

ament_target_dependencies(spot_sm rclcpp mini_interfaces std_srvs std_msgs)
ament_target_dependencies(teleop_node rclcpp mini_interfaces sensor_msgs geometry_msgs std_srvs std_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(
spot_sm
# Eigen3::Eigen
${PROJECT_NAME}
)

target_link_libraries(
teleop_node
# Eigen3::Eigen
${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  spot_sm
  teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mini_ros2/spot_pybullet_interface
  mini_ros2/spot_real_interface
  mini_ros2/servo_calibration
  mini_ros2/sensor_interface
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

ament_package()
