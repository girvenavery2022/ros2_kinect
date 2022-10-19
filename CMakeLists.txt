cmake_minimum_required(VERSION 3.8)
project(ros2_kinect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_library(LIBFREENECT_LIBRARY
  NAMES freenect
  PATHS ${LIBFREENECT_LIBRARY_DIRS}
)

add_executable(kinect_node 
  src/kinect_node.cpp
  src/kinect_node_main.cpp
)

ament_target_dependencies(
  kinect_node
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_geometry
  Boost
)

target_link_libraries(kinect_node freenect_sync
  ${Boost_LIBRARY}
  ${LIBFREENECT_LIBRARY})

target_include_directories(kinect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${LIBFREENECT_INCLUDE_DIRS})

target_compile_features(kinect_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS kinect_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
