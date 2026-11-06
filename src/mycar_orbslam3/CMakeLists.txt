cmake_minimum_required(VERSION 3.16)
project(mycar_orbslam3)

find_package(ament_cmake REQUIRED)
get_filename_component(WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(ORB_SLAM3_ROOT "${WORKSPACE}/.orbslam3/ORB_SLAM3" CACHE PATH "ORB-SLAM3 source and library")
set(ORB_PANGOLIN_PREFIX "${WORKSPACE}/.orbslam3/prefix" CACHE PATH "Local Pangolin installation")
configure_file(config/dependencies.yaml.in dependencies.yaml @ONLY)

# 可选测试模块：尚未准备视觉依赖时，原有ROS工程仍可正常编译。
if(EXISTS "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(OpenCV 4 REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Pangolin REQUIRED PATHS "${ORB_PANGOLIN_PREFIX}/lib/cmake/Pangolin" NO_DEFAULT_PATH)

  add_executable(rgbd_slam src/rgbd_slam.cpp)
  target_compile_features(rgbd_slam PRIVATE cxx_std_17)
  target_compile_options(rgbd_slam PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(rgbd_slam SYSTEM PRIVATE
    "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include"
    "${ORB_SLAM3_ROOT}/include/CameraModels" "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
    ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
  ament_target_dependencies(rgbd_slam rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs
    message_filters cv_bridge tf2_ros)
  target_link_libraries(rgbd_slam "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
    Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
  # 使用本机编译的库；NX上重新编译，不复制PC的x86二进制，不修改系统库路径。
  set_target_properties(rgbd_slam PROPERTIES INSTALL_RPATH
    "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;${ORB_PANGOLIN_PREFIX}/lib")
  install(TARGETS rgbd_slam DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "ORB-SLAM3 not built: rgbd_slam skipped. Run bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_orb_slam3.sh, then rebuild mycar_orbslam3.")
endif()

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME} PATTERN "*.in" EXCLUDE)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/dependencies.yaml" DESTINATION share/${PROJECT_NAME}/config)
install(FILES UPSTREAM.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
