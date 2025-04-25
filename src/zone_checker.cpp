#include "ros_geofence/zone_checker.hpp"

namespace ros_geofence
{
ZoneChecker::ZoneChecker()
: Node("zone_checker"), polygon_loaded_(false)
{
  // Оголошення параметра для шляху до GeoJSON
  this->declare_parameter<std::string>("geojson_file", "zone.geojson");

  // Завантаження полігону з GeoJSON
  polygon_loaded_ = load_geojson_zone();

  // Підписка на топік GPS
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10, std::bind(&ZoneChecker::gps_callback, this, std::placeholders::_1));

  // Публікатор для результату
  zone_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_in_zone", 10);

  RCLCPP_INFO(this->get_logger(), "Zone checker node started");
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

    // Витягуємо координати полігону
    auto coordinates = geojson_data["features"][0]["geometry"]["coordinates"][0];
    zone_polygon_.clear();

    for (const auto& coord : coordinates) {
      Point p;
      p.x = coord[0].get<double>();  // Довгота
      p.y = coord[1].get<double>();  // Широта
      zone_polygon_.push_back(p);
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
  // Алгоритм ray-casting для перевірки, чи точка всередині полігону
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

void ZoneChecker::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!polygon_loaded_) {
    RCLCPP_WARN(this->get_logger(), "No valid zone loaded");
    return;
  }

  // Створюємо точку з координат робота
  Point robot_point;
  robot_point.x = msg->longitude;
  robot_point.y = msg->latitude;

  // Перевіряємо, чи точка всередині полігону
  bool is_in_zone = point_in_polygon(robot_point, zone_polygon_);

  // Публікуємо результат
  auto zone_msg = std_msgs::msg::Bool();
  zone_msg.data = is_in_zone;
  zone_pub_->publish(zone_msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Robot at (%f, %f) - In zone: %s",
    msg->longitude, msg->latitude, is_in_zone ? "true" : "false");
}

}  // namespace ros_geofence

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_geofence::ZoneChecker>());
  rclcpp::shutdown();
  return 0;
}