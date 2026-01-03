/**
 * @file zone_checker.cpp
 * @brief Реалізація ноди ZoneChecker для перевірки знаходження робота в геозоні.
 */

#include "ros_geofence/zone_checker.hpp"
#include <cmath>

namespace ros_geofence
{

ZoneChecker::ZoneChecker()
: Node("zone_checker"), polygon_loaded_(false), proj_(nullptr)
{
  // Оголошення параметрів з значеннями за замовчуванням
  this->declare_parameter<std::string>("geojson_file", "zone.geojson");
  this->declare_parameter<double>("costmap_resolution", 0.1);
  this->declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
  this->declare_parameter<std::string>("geofence_zone_topic", "/geofence_zone");

  // Завантаження полігону зони з GeoJSON файлу
  polygon_loaded_ = load_geojson_zone();

  // Підписка на GPS топік для отримання позиції робота
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10, std::bind(&ZoneChecker::gps_callback, this, std::placeholders::_1));

  // Публікатор булевого результату: чи робот в зоні
  zone_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_in_zone", 10);

  // Публікатор costmap для інтеграції з Nav2
  std::string costmap_topic;
  this->get_parameter("costmap_topic", costmap_topic);
  costmap_pub_ = this->create_publisher<nav2_msgs::msg::Costmap>(costmap_topic, 10);

  // Публікатор полігону зони (для Mapviz плагіну Polygon)
  std::string geofence_zone_topic;
  this->get_parameter("geofence_zone_topic", geofence_zone_topic);
  geofence_zone_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(geofence_zone_topic, 10);

  // Публікатор маркера зони (для Mapviz плагіну Marker)
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/geofence_marker", 10);

  // Публікуємо візуалізацію одразу після ініціалізації
  if (polygon_loaded_) {
    publish_costmap();
    publish_geofence_zone();
    publish_geofence_marker();
  }

  RCLCPP_INFO(this->get_logger(), "Zone checker node started");
}

/**
 * @brief Завантажує полігон зони з GeoJSON файлу.
 * 
 * Автоматично визначає UTM зону на основі координат полігону
 * та конвертує всі точки в метричну систему координат.
 * 
 * @return true якщо завантаження успішне, false при помилці.
 */
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

    // Витягуємо координати першого полігону з GeoJSON
    auto coordinates = geojson_data["features"][0]["geometry"]["coordinates"][0];
    zone_polygon_.clear();
    zone_polygon_utm_.clear();

    // Обчислюємо центроїд полігону для визначення UTM зони
    double avg_lon = 0.0, avg_lat = 0.0;
    int count = 0;
    for (const auto& coord : coordinates) {
      avg_lon += coord[0].get<double>();
      avg_lat += coord[1].get<double>();
      count++;
    }
    avg_lon /= count;
    avg_lat /= count;

    // Визначаємо номер UTM зони за довготою (кожна зона 6 градусів)
    int zone_number = static_cast<int>(std::floor((avg_lon + 180.0) / 6.0)) + 1;
    bool is_northern = avg_lat >= 0;
    // EPSG код: 326XX для північної півкулі, 327XX для південної
    int epsg_code = is_northern ? 32600 + zone_number : 32700 + zone_number;
    std::string utm_zone = "EPSG:" + std::to_string(epsg_code);

    // Ініціалізація PROJ трансформації WGS84 -> UTM
    proj_ = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", utm_zone.c_str(), nullptr);
    if (!proj_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize projection for %s", utm_zone.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Automatically determined UTM zone: %s", utm_zone.c_str());

    // Завантажуємо та конвертуємо координати полігону
    for (const auto& coord : coordinates) {
      Point geo_p, utm_p;
      geo_p.x = coord[0].get<double>();  // Довгота (longitude)
      geo_p.y = coord[1].get<double>();  // Широта (latitude)
      zone_polygon_.push_back(geo_p);

      // Конвертація географічних координат в UTM (метри)
      PJ_COORD coord_in = proj_coord(geo_p.y, geo_p.x, 0, 0);  // lat, lon порядок для PROJ
      PJ_COORD coord_out = proj_trans(proj_, PJ_FWD, coord_in);
      utm_p.x = coord_out.xy.x;  // UTM Easting (метри)
      utm_p.y = coord_out.xy.y;  // UTM Northing (метри)
      zone_polygon_utm_.push_back(utm_p);
    }

    RCLCPP_INFO(this->get_logger(), "GeoJSON loaded successfully");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading GeoJSON: %s", e.what());
    return false;
  }
}

/**
 * @brief Перевіряє чи точка знаходиться всередині полігону.
 * 
 * Використовує алгоритм ray-casting: проводить промінь від точки
 * та рахує кількість перетинів з ребрами полігону.
 * Непарна кількість перетинів означає що точка всередині.
 * 
 * @param p Точка для перевірки.
 * @param polygon Вектор вершин полігону.
 * @return true якщо точка всередині полігону.
 */
bool ZoneChecker::point_in_polygon(const Point& p, const std::vector<Point>& polygon)
{
  int n = polygon.size();
  bool inside = false;

  // Проходимо по всіх ребрах полігону
  for (int i = 0, j = n - 1; i < n; j = i++) {
    // Перевіряємо чи горизонтальний промінь від точки перетинає ребро
    if (((polygon[i].y > p.y) != (polygon[j].y > p.y)) &&
        (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) /
                 (polygon[j].y - polygon[i].y) + polygon[i].x)) {
      inside = !inside;  // Інвертуємо при кожному перетині
    }
  }

  return inside;
}

/**
 * @brief Публікує costmap зони для інтеграції з Nav2.
 * 
 * Створює растрову карту де:
 * - 0 (вільно) - точки всередині дозволеної зони
 * - 254 (lethal) - точки поза зоною
 */
void ZoneChecker::publish_costmap()
{
  if (!polygon_loaded_) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish costmap: no valid zone loaded");
    return;
  }

  double resolution;
  this->get_parameter("costmap_resolution", resolution);

  // Визначаємо bounding box полігону в UTM координатах
  double min_x = zone_polygon_utm_[0].x, max_x = zone_polygon_utm_[0].x;
  double min_y = zone_polygon_utm_[0].y, max_y = zone_polygon_utm_[0].y;
  for (const auto& p : zone_polygon_utm_) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  // Обмежуємо розмір costmap (максимум 500x500 клітинок)
  int width = std::min(static_cast<int>((max_x - min_x) / resolution) + 1, 500);
  int height = std::min(static_cast<int>((max_y - min_y) / resolution) + 1, 500);

  // Формуємо повідомлення costmap
  auto costmap_msg = nav2_msgs::msg::Costmap();
  costmap_msg.header.frame_id = "map";
  costmap_msg.header.stamp = this->get_clock()->now();
  costmap_msg.metadata.layer = "geofence";
  costmap_msg.metadata.resolution = resolution;
  // Origin - нижній лівий кут карти
  costmap_msg.metadata.origin.position.x = min_x;
  costmap_msg.metadata.origin.position.y = min_y;
  costmap_msg.metadata.origin.position.z = 0.0;
  costmap_msg.metadata.origin.orientation.w = 1.0;
  costmap_msg.metadata.size_x = width;
  costmap_msg.metadata.size_y = height;

  // Заповнюємо карту: 254 (lethal) за замовчуванням, 0 (free) всередині зони
  costmap_msg.data.resize(width * height, 254);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Point p;
      p.x = min_x + x * resolution;
      p.y = min_y + y * resolution;
      if (point_in_polygon(p, zone_polygon_utm_)) {
        costmap_msg.data[y * width + x] = 0;  // Вільна клітинка
      }
    }
  }

  costmap_pub_->publish(costmap_msg);
  RCLCPP_INFO(this->get_logger(), "Published costmap to %s", costmap_pub_->get_topic_name());
}

/**
 * @brief Публікує полігон зони як PolygonStamped повідомлення.
 * 
 * Координати публікуються в географічній системі (WGS84)
 * для відображення на мапі.
 */
void ZoneChecker::publish_geofence_zone()
{
  if (!polygon_loaded_) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish geofence zone: no valid zone loaded");
    return;
  }

  auto polygon_msg = geometry_msgs::msg::PolygonStamped();
  polygon_msg.header.frame_id = "wgs84";  // Географічна система координат
  polygon_msg.header.stamp = this->get_clock()->now();

  // Додаємо всі вершини полігону
  for (const auto& point : zone_polygon_) {
    geometry_msgs::msg::Point32 p;
    p.x = point.x;  // Довгота
    p.y = point.y;  // Широта
    p.z = 0.0;
    polygon_msg.polygon.points.push_back(p);
  }

  geofence_zone_pub_->publish(polygon_msg);
  RCLCPP_INFO(this->get_logger(), "Published geofence zone to %s", geofence_zone_pub_->get_topic_name());
}

/**
 * @brief Публікує маркер зони для візуалізації в Mapviz.
 * 
 * Створює LINE_STRIP маркер що відображає межі зони
 * червоною лінією на карті.
 */
void ZoneChecker::publish_geofence_marker()
{
  if (!polygon_loaded_) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish geofence marker: no valid zone loaded");
    return;
  }

  auto marker_msg = visualization_msgs::msg::Marker();
  marker_msg.header.frame_id = "wgs84";  // Географічна система координат
  marker_msg.header.stamp = this->get_clock()->now();
  marker_msg.ns = "geofence";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.scale.x = 0.0001;  // Товщина лінії (в градусах для wgs84)
  marker_msg.color.r = 1.0;     // Червоний колір
  marker_msg.color.a = 1.0;     // Повна непрозорість

  // Додаємо всі вершини полігону
  for (const auto& point : zone_polygon_) {
    geometry_msgs::msg::Point p;
    p.x = point.x;  // Довгота
    p.y = point.y;  // Широта
    p.z = 0.0;
    marker_msg.points.push_back(p);
  }

  // Замикаємо полігон - з'єднуємо останню точку з першою
  if (!zone_polygon_.empty()) {
    geometry_msgs::msg::Point p;
    p.x = zone_polygon_[0].x;
    p.y = zone_polygon_[0].y;
    p.z = 0.0;
    marker_msg.points.push_back(p);
  }

  marker_pub_->publish(marker_msg);
  RCLCPP_INFO(this->get_logger(), "Published geofence marker to %s", marker_pub_->get_topic_name());
}

/**
 * @brief Callback для обробки GPS повідомлень.
 * 
 * Конвертує GPS координати в UTM, перевіряє чи робот в зоні
 * та публікує результат.
 * 
 * @param msg Повідомлення з GPS координатами робота.
 */
void ZoneChecker::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!polygon_loaded_) {
    RCLCPP_WARN(this->get_logger(), "No valid zone loaded");
    return;
  }

  // Конвертуємо GPS координати робота в UTM
  PJ_COORD coord_in = proj_coord(msg->latitude, msg->longitude, 0, 0);
  PJ_COORD coord_out = proj_trans(proj_, PJ_FWD, coord_in);
  Point robot_point;
  robot_point.x = coord_out.xy.x;  // UTM Easting
  robot_point.y = coord_out.xy.y;  // UTM Northing

  // Перевіряємо чи робот всередині зони
  bool is_in_zone = point_in_polygon(robot_point, zone_polygon_utm_);

  // Публікуємо булевий результат
  auto zone_msg = std_msgs::msg::Bool();
  zone_msg.data = is_in_zone;
  zone_pub_->publish(zone_msg);

  // Оновлюємо візуалізацію маркера
  publish_geofence_marker();

  RCLCPP_INFO(
    this->get_logger(),
    "Robot at (%f, %f) - In zone: %s",
    msg->longitude, msg->latitude, is_in_zone ? "true" : "false");
}

}  // namespace ros_geofence

/**
 * @brief Точка входу програми.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_geofence::ZoneChecker>());
  rclcpp::shutdown();
  return 0;
}