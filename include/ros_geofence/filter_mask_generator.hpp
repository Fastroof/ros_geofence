#pragma once

// Standard library
#include <fstream>
#include <string>
#include <vector>
#include <memory>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// Third-party
#include <nlohmann/json.hpp>
#include <proj.h>

namespace ros_geofence
{

class FilterMaskGenerator : public rclcpp::Node
{
public:
  FilterMaskGenerator();
  ~FilterMaskGenerator();

  bool generate();

private:
  struct Point {
    double x;
    double y;
  };

  struct GridBounds {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
  };

  bool load_geojson_zone();
  bool point_in_polygon(const Point& p, const std::vector<Point>& polygon);
  GridBounds calculate_bounds();
  bool generate_pgm_mask(const std::string& pgm_path, const std::string& yaml_path);
  void write_pgm_file(const std::string& path, const std::vector<uint8_t>& data, 
                      int width, int height);
  void write_yaml_file(const std::string& path, int width, int height, 
                       double resolution, double origin_x, double origin_y);

  std::vector<Point> zone_polygon_;
  std::vector<Point> zone_polygon_utm_;
  PJ* proj_;
  
  double resolution_;
  double padding_;
  std::string geojson_file_;
  std::string output_pgm_;
  std::string output_yaml_;
};

}  // namespace ros_geofence
