cmake_minimum_required(VERSION 3.8)
project(robo_swarm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)

# 2. Install Directories (This is the important part!)
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

# 3. Testing (Standard ROS 2 stuff, keep it)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()