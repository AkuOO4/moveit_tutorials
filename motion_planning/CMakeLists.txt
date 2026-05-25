cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)



# Add executables
add_executable(planning_pipeline src/planning_pipeline.cpp)
add_executable(planning_api src/planning_api.cpp)

# Include directories
target_include_directories(planning_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(planning_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Compile features
target_compile_features(planning_pipeline PUBLIC c_std_99 cxx_std_17)
target_compile_features(planning_api PUBLIC c_std_99 cxx_std_17)

# Dependencies
ament_target_dependencies(
  planning_pipeline
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "moveit_msgs"
  "geometry_msgs"
  "shape_msgs"
)
ament_target_dependencies(
  planning_api
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "moveit_msgs"
  "geometry_msgs"
  "shape_msgs"
)



install(TARGETS planning_pipeline planning_api
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
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
