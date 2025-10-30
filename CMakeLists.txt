cmake_minimum_required(VERSION 3.8)
project(hebi_okblab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/usr/include/eigen3)

# find dependencies
find_package(ament_cmake REQUIRED)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


# if(NOT EIGEN3_INCLUDE_DIR)
#   find_path(EIGEN3_INCLUDE_DIR Eigen/Core
#     PATHS /usr/include/eigen3
#           # /usr/local/include/eigen3
#   )
# endif()

# if(EIGEN3_INCLUDE_DIR)
#   message(STATUS "Found Eigen3 in ${EIGEN3_INCLUDE_DIR}")
#   include_directories(${EIGEN3_INCLUDE_DIR})
# else()
#   message(FATAL_ERROR "Eigen3 not found. Please check your installation.")
# endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
find_package(hebi_cpp_api)
# find_package(Eigen3 REQUIRED)

add_executable(command_position src/command_position.cpp)
ament_target_dependencies(command_position
  rclcpp
  std_msgs
  hebi_cpp_api
  # Eigen3
)
# target_include_directories(command_position PRIVATE
#   ${EIGEN3_INCLUDE_DIR}
# )

install(TARGETS
  command_position
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
