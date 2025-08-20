cmake_minimum_required(VERSION 3.8)
project(tros_runtime_stats)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -ggdb -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/runtime_stats.cpp
)

add_executable(${PROJECT_NAME}_sample sample/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}_sample PRIVATE ${PROJECT_NAME})

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

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/)

install(TARGETS ${PROJECT_NAME}_sample
  DESTINATION lib/${PROJECT_NAME}/)

install(FILES
  include/tros_runtime_stats/runtime_stats.h
  DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
