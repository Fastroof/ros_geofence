#pragma once

// Standard library
#include <fstream>
#include <string>
#include <vector>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Third-party
#include <nlohmann/json.hpp>
#include <proj.h>

namespace ros_geofence
{

class ZoneChecker : public rclcpp::Node
{
public:
  ZoneChecker();
  ~ZoneChecker();

private:
  struct Point {
    double x;
    double y;
  };

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  bool load_geojson_zone();
  bool point_in_polygon(const Point& p, const std::vector<Point>& polygon);
  void publish_geofence_marker();

  std::vector<Point> zone_polygon_;
  std::vector<Point> zone_polygon_utm_;
  bool polygon_loaded_;
  PJ* proj_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zone_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  visualization_msgs::msg::Marker marker_msg_;
};

}  // namespace ros_geofence
