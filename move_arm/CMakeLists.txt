cmake_minimum_required(VERSION 3.8)
project(move_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(move_arm_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)#for rotating plant
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(move_arm_node src/move_arm.cpp)
ament_target_dependencies(move_arm_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  moveit_ros_planning_interface
  move_arm_msgs
  shape_msgs#ADDED 04.08
  tf2_ros#ADDED 04.08
  geometric_shapes#ADDED 04.08
)

add_executable(plant_loader_node src/plant_loader_node.cpp)
ament_target_dependencies(plant_loader_node
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  ament_index_cpp
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)#ADDED 04.08

install(TARGETS
  move_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS plant_loader_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
