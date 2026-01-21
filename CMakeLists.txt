cmake_minimum_required(VERSION 3.8)
project(ros_geofence)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Знайдіть залежності
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PROJ REQUIRED)

# Додайте виконуваний файл zone_checker
add_executable(zone_checker src/zone_checker.cpp)
target_include_directories(zone_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(zone_checker PUBLIC cxx_std_17)
ament_target_dependencies(zone_checker
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  PROJ
)

# Додайте виконуваний файл fake_robot
add_executable(fake_robot src/fake_robot.cpp)
target_include_directories(fake_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fake_robot PUBLIC cxx_std_17)
ament_target_dependencies(fake_robot
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# Встановлення виконуваних файлів
install(TARGETS zone_checker fake_robot
  DESTINATION lib/${PROJECT_NAME})

# Встановлення конфігураційних файлів
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Встановлення launch файлів
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()