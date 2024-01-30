cmake_minimum_required(VERSION 3.5)
project(mpc_package)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(/usr/include
                    include/mpc_package
                    include/mpc_package/MPC
                    include/mpc_package/Eigen
                    include/mpc_package/Json/include)

add_library(mpc include/mpc_package/MPC/MPC.cpp)
target_link_libraries(mpc ipopt)

add_executable(mpc_node src/mpc_node.cpp)
ament_target_dependencies(mpc_node rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(mpc_node mpc)

add_executable(get_score src/get_score.cpp)
ament_target_dependencies(get_score rclcpp std_msgs geometry_msgs)

install(
  TARGETS 
  mpc_node
  get_score
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
