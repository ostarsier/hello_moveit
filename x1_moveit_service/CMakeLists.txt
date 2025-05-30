cmake_minimum_required(VERSION 3.8)
project(x1_moveit_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(x1_moveit_proto REQUIRED)

add_executable(x1_moveit_service src/x1_moveit_service.cpp)
target_include_directories(x1_moveit_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(x1_moveit_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  x1_moveit_service
  "moveit_ros_planning_interface"
  "rclcpp"
)

# 添加x1_moveit_rviz可执行文件
add_executable(x1_moveit_rviz src/x1_moveit_rviz.cpp)
target_include_directories(x1_moveit_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(x1_moveit_rviz PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  x1_moveit_rviz
  "moveit_ros_planning_interface"
  "rclcpp"
  "x1_moveit_proto"
)

# 添加x1_moveit_ik可执行文件
add_executable(x1_moveit_ik src/x1_moveit_ik.cpp)
target_include_directories(x1_moveit_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(x1_moveit_ik PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  x1_moveit_ik
  "moveit_ros_planning_interface"
  "rclcpp"
)

# 添加x1_moveit_ik_service可执行文件
add_executable(x1_moveit_ik_service src/x1_moveit_ik_service.cpp)
target_include_directories(x1_moveit_ik_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(x1_moveit_ik_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  x1_moveit_ik_service
  "moveit_ros_planning_interface"
  "rclcpp"
  "x1_moveit_proto"
)

# 添加x1_moveit_rviz_http_service可执行文件
add_executable(x1_moveit_rviz_http_service src/x1_moveit_rviz_http_service.cpp)
target_include_directories(x1_moveit_rviz_http_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(x1_moveit_rviz_http_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  x1_moveit_rviz_http_service
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS x1_moveit_service x1_moveit_rviz x1_moveit_ik x1_moveit_ik_service x1_moveit_rviz_http_service
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
