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
find_package(nlohmann_json REQUIRED)

# Додайте виконуваний файл
add_executable(zone_checker src/zone_checker.cpp)
target_include_directories(zone_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(zone_checker PUBLIC c_std_99 cxx_std_17)  # C++17
ament_target_dependencies(zone_checker
  rclcpp
  sensor_msgs
  std_msgs
  nlohmann_json
)

# Встановлення виконуваного файлу
install(TARGETS zone_checker
  DESTINATION lib/${PROJECT_NAME})

# Встановлення GeoJSON
install(FILES config/zone.geojson
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()