cmake_minimum_required(VERSION 3.5)
project(rtv_nav2_plan_optimizer)

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup

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
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# nav2_package()

include_directories(
  include
  ${CERES_INCLUDES}
)

set(executable_name optimizer_server)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)

add_library(${library_name}
  src/rtv_nav2_plan_optimizer.cpp
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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
)

add_library(ceres_costaware_smoother SHARED plugins/ceres_costaware_smoother.cpp)
target_link_libraries(ceres_costaware_smoother ${CERES_LIBRARIES})
ament_target_dependencies(ceres_costaware_smoother ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(ceres_costaware_smoother PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(plugins/test)
endif()

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

install(
  TARGETS
    ceres_costaware_smoother
    ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  ceres_costaware_smoother
  ${library_name})
pluginlib_export_plugin_description_file(rtv_nav2_plan_optimizer plugins.xml)

ament_package()
