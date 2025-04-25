#ifndef ZONE_CHECKER_HPP_
#define ZONE_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nlohmann/json.hpp>
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

  std::vector<Point> zone_polygon_;
  bool polygon_loaded_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zone_pub_;
};
}  // namespace ros_geofence

#endif  // ZONE_CHECKER_HPP_