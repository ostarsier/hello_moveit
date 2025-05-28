cmake_minimum_required(VERSION 3.22)
project(x1_moveit_config)

# 添加编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# 查找所需的依赖包
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 查找hiredis库
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIREDIS REQUIRED hiredis)

# 添加包含目录
include_directories(include)

# 创建硬件接口库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/x1_system_interface.cpp
)

# 设置目标属性和链接库
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${HIREDIS_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${HIREDIS_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 安装目标
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装插件描述文件
pluginlib_export_plugin_description_file(hardware_interface x1_hardware_interface_plugin.xml)

# 安装launch和config文件
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 导出包
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
