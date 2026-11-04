cmake_minimum_required(VERSION 3.8)
project(mycar_base)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
add_library(mycar_hardware SHARED src/mycar_system.cpp)
target_compile_features(mycar_hardware PUBLIC cxx_std_17)
target_include_directories(mycar_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mycar_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface mycar_hardware.xml)

install(TARGETS mycar_hardware
  EXPORT export_mycar_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
# 升级时清除已移除节点的旧安装文件，避免旧工作空间残留可执行程序。
install(CODE [[
  file(REMOVE
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mycar_base/scan_deskew"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/mycar_base/config/deskew.yaml"
    "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/mycar_base/config/laser_odom.yaml")
]])
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_mycar_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
