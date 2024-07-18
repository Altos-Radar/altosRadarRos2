cmake_minimum_required(VERSION 3.8)
project(altosradar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

include_directories(
  include 
  ${PCL_INCLUDE_DIRS}
)
ament_export_include_directories(include)

add_executable(altosRadarParse src/altosRadarParse.cpp)

ament_target_dependencies(altosRadarParse 
  rclcpp 
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(altosRadarParse ${PCL_LIBRARIES})

install(TARGETS
  altosRadarParse
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
