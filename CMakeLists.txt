cmake_minimum_required(VERSION 3.8)
project(bag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(modaq_messages REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(bag_recorder_node src/bag_recorder_node.cpp)
ament_target_dependencies(bag_recorder_node rclcpp rosbag2_cpp rosbag2_transport rosbag2_storage modaq_messages)

target_link_libraries(bag_recorder_node "${cpp_typesupport_target}" )

ament_export_dependencies()

install(TARGETS
bag_recorder_node
DESTINATION lib/${PROJECT_NAME})

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
