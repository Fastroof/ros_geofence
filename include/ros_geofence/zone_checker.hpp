#ifndef ZONE_CHECKER_HPP_
#define ZONE_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nlohmann/json.hpp>
#include <proj.h>
#include <fstream>
#include <string>
#include <vector>

namespace ros_geofence
{
class ZoneChecker : public rclcpp::Node
{
public:
  ZoneChecker();

private:
  struct Point {
    double x;
    double y;
  };

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  bool load_geojson_zone();
  bool point_in_polygon(const Point& p, const std::vector<Point>& polygon);
  void publish_costmap();
  void publish_geofence_zone();
  void publish_geofence_marker();

  std::vector<Point> zone_polygon_;  // У географічних координатах
  std::vector<Point> zone_polygon_utm_;  // У UTM координатах
  bool polygon_loaded_;
  PJ* proj_;  // Об'єкт для проекції

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zone_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr geofence_zone_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
}  // namespace ros_geofence

#endif  // ZONE_CHECKER_HPP_