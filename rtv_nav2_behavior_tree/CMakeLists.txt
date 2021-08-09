cmake_minimum_required(VERSION 3.5)
project(rtv_nav2_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rtv_nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# nav2_package()

include_directories(
  include
)

set(executable_name optimizer_server)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  rtv_nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  nav2_core
  pluginlib
  nav2_behavior_tree
)

add_library(rtv_nav2_optimize_path_action_bt_node SHARED plugins/optimize_path_action.cpp)
ament_target_dependencies(rtv_nav2_optimize_path_action_bt_node ${dependencies})
target_compile_definitions(rtv_nav2_optimize_path_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(rtv_nav2_resetter_node_bt_node SHARED plugins/resetter_node.cpp)
ament_target_dependencies(rtv_nav2_resetter_node_bt_node ${dependencies})
target_compile_definitions(rtv_nav2_resetter_node_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(rtv_nav2_fallback_pipeline_bt_node SHARED plugins/fallback_pipeline.cpp)
ament_target_dependencies(rtv_nav2_fallback_pipeline_bt_node ${dependencies})
target_compile_definitions(rtv_nav2_fallback_pipeline_bt_node PRIVATE BT_PLUGIN_EXPORT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(plugins/test)
endif()

install(
  TARGETS
    rtv_nav2_optimize_path_action_bt_node
    rtv_nav2_resetter_node_bt_node
    rtv_nav2_fallback_pipeline_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  rtv_nav2_optimize_path_action_bt_node
  rtv_nav2_resetter_node_bt_node
  rtv_nav2_fallback_pipeline_bt_node
)

ament_package()
