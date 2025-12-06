cmake_minimum_required(VERSION 3.8)
project(mrs_bumper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_STANDARD LESS 17)
  message(FATAL_ERROR "This project requires C++17 or higher")
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Compile options
add_compile_options(-Wall -Wextra)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  image_geometry
  tf2_ros
  tf2_geometry_msgs
  mrs_lib
  mrs_msgs
  pcl_conversions
  pcl_ros
  pcl_msgs
)

# Build library
add_library(mrs_bumper_lib SHARED
  src/bumper_nodelet.cpp
)

ament_target_dependencies(mrs_bumper_lib ${dependencies})

target_include_directories(mrs_bumper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mrs_lib_INCLUDE_DIRS}
)

target_link_libraries(mrs_bumper_lib
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Register component
rclcpp_components_register_node(mrs_bumper_lib
  PLUGIN "mrs_bumper::Bumper"
  EXECUTABLE mrs_bumper_node
)

# Build executable
add_executable(histogram_displayer
  src/histogram_displayer.cpp
)

ament_target_dependencies(histogram_displayer
  rclcpp
  mrs_lib
  mrs_msgs
)

target_include_directories(histogram_displayer PRIVATE
  ${mrs_lib_INCLUDE_DIRS}
)

target_link_libraries(histogram_displayer
  ${OpenCV_LIBRARIES}
)

# Install
install(TARGETS
  mrs_bumper_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  histogram_displayer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
