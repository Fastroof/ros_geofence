#include <cmath>
#include "ros_geofence/zone_checker.hpp"

namespace ros_geofence
{

ZoneChecker::ZoneChecker()
: Node("zone_checker"),
  polygon_loaded_(false),
  proj_(nullptr)
{
  this->declare_parameter<std::string>("geojson_file", "zone.geojson");
  this->declare_parameter<std::string>("gps_topic", "/gps/fix");

  polygon_loaded_ = load_geojson_zone();

  const std::string gps_topic = this->get_parameter("gps_topic").as_string();
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, rclcpp::SensorDataQoS(), std::bind(&ZoneChecker::gps_callback, this, std::placeholders::_1));
  zone_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_in_zone", 2);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/geofence_marker", 1);

  // Initialize geofence marker once
  if (polygon_loaded_) {
    marker_msg_.header.frame_id = "wgs84";
    marker_msg_.ns = "geofence";
    marker_msg_.id = 0;
    marker_msg_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_msg_.scale.x = 0.0001;
    marker_msg_.color.r = 1.0;
    marker_msg_.color.a = 1.0;

    for (const auto& point : zone_polygon_) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      marker_msg_.points.push_back(p);
    }

    if (!zone_polygon_.empty()) {
      geometry_msgs::msg::Point p;
      p.x = zone_polygon_[0].x;
      p.y = zone_polygon_[0].y;
      p.z = 0.0;
      marker_msg_.points.push_back(p);
    }
  }

  marker_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ZoneChecker::publish_geofence_marker, this));

  RCLCPP_INFO(this->get_logger(), "Zone checker node started");
}

ZoneChecker::~ZoneChecker()
{
  if (proj_) {
    proj_destroy(proj_);
    proj_ = nullptr;
  }
}

bool ZoneChecker::load_geojson_zone()
{
  std::string geojson_file;
  this->get_parameter("geojson_file", geojson_file);

  try {
    std::ifstream file(geojson_file);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GeoJSON file: %s", geojson_file.c_str());
      return false;
    }

    nlohmann::json geojson_data;
    file >> geojson_data;

    auto coordinates = geojson_data["features"][0]["geometry"]["coordinates"][0];
    zone_polygon_.clear();
    zone_polygon_utm_.clear();

    double avg_lon = 0.0, avg_lat = 0.0;
    int count = 0;
    for (const auto& coord : coordinates) {
      avg_lon += coord[0].get<double>();
      avg_lat += coord[1].get<double>();
      count++;
    }
    avg_lon /= count;
    avg_lat /= count;

    int zone_number = static_cast<int>(std::floor((avg_lon + 180.0) / 6.0)) + 1;
    bool is_northern = avg_lat >= 0;
    int epsg_code = is_northern ? 32600 + zone_number : 32700 + zone_number;
    std::string utm_zone = "EPSG:" + std::to_string(epsg_code);

    proj_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", utm_zone.c_str(), nullptr);
    if (!proj_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize projection for %s", utm_zone.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Automatically determined UTM zone: %s", utm_zone.c_str());

    for (const auto& coord : coordinates) {
      Point geo_p, utm_p;
      geo_p.x = coord[0].get<double>();
      geo_p.y = coord[1].get<double>();
      zone_polygon_.push_back(geo_p);

      PJ_COORD coord_in = proj_coord(geo_p.y, geo_p.x, 0, 0);
      PJ_COORD coord_out = proj_trans(proj_, PJ_FWD, coord_in);
      utm_p.x = coord_out.xy.x;
      utm_p.y = coord_out.xy.y;
      zone_polygon_utm_.push_back(utm_p);
    }

    RCLCPP_INFO(this->get_logger(), "GeoJSON loaded successfully");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading GeoJSON: %s", e.what());
    return false;
  }
}

bool ZoneChecker::point_in_polygon(const Point& p, const std::vector<Point>& polygon)
{
  int n = polygon.size();
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon[i].y > p.y) != (polygon[j].y > p.y)) &&
        (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) /
                 (polygon[j].y - polygon[i].y) + polygon[i].x)) {
      inside = !inside;
    }
  }

  return inside;
}

void ZoneChecker::publish_geofence_marker()
{
  if (polygon_loaded_) {
    marker_msg_.header.stamp = this->get_clock()->now();
    marker_pub_->publish(marker_msg_);
    RCLCPP_DEBUG(this->get_logger(), "Published geofence marker to %s", marker_pub_->get_topic_name());
  } else {
    RCLCPP_WARN(this->get_logger(), "Cannot publish geofence marker: no valid zone loaded");
  }
}

void ZoneChecker::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!polygon_loaded_) {
    RCLCPP_WARN(this->get_logger(), "No valid zone loaded");
    return;
  }

  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "GPS does not have a valid fix (status: %d)", msg->status.status);
    return;
  }

  PJ_COORD coord_in = proj_coord(msg->latitude, msg->longitude, 0, 0);
  PJ_COORD coord_out = proj_trans(proj_, PJ_FWD, coord_in);
  Point robot_point;
  robot_point.x = coord_out.xy.x;
  robot_point.y = coord_out.xy.y;

  bool is_in_zone = point_in_polygon(robot_point, zone_polygon_utm_);

  auto zone_msg = std_msgs::msg::Bool();
  zone_msg.data = is_in_zone;
  zone_pub_->publish(zone_msg);
}

}  // namespace ros_geofence

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_geofence::ZoneChecker>());
  rclcpp::shutdown();
  return 0;
}