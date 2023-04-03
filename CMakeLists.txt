cmake_minimum_required(VERSION 3.8)
project(hkcamera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)     
include_directories(/usr/include/opencv4)   
include_directories(/opt/ros/humble/include/cv_bridge)     
include_directories(/opt/ros/humble/include/image_transport)                     
# find_package(MvCameraControl REQUIRED)


add_executable(single_cam_node src/single_cam_node.cpp)
add_executable(test1 src/test1.cpp)
target_link_libraries(single_cam_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${OpenCV_LIBS}
  MvCameraControl
)
target_link_libraries(test1
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(single_cam_node rclcpp sensor_msgs image_transport OpenCV cv_bridge)
ament_target_dependencies(test1 rclcpp sensor_msgs image_transport OpenCV cv_bridge)

install(TARGETS
  single_cam_node
  test1
  DESTINATION lib/${PROJECT_NAME}
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
