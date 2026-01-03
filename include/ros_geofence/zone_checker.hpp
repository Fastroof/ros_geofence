/**
 * @file zone_checker.hpp
 * @brief Заголовковий файл ноди ZoneChecker для перевірки знаходження робота в зоні.
 * 
 * Нода завантажує полігон зони з GeoJSON файлу, конвертує координати в UTM проекцію
 * та перевіряє чи GPS позиція робота знаходиться в межах зони.
 */

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

/**
 * @class ZoneChecker
 * @brief ROS 2 нода для перевірки знаходження робота в географічній зоні.
 * 
 * Функціональність:
 * - Завантаження полігону зони з GeoJSON
 * - Автоматичне визначення UTM зони
 * - Перевірка позиції робота алгоритмом ray-casting
 * - Публікація результатів та візуалізація в Mapviz
 */
class ZoneChecker : public rclcpp::Node
{
public:
  /**
   * @brief Конструктор. Ініціалізує параметри, завантажує зону та створює pub/sub.
   */
  ZoneChecker();

private:
  /**
   * @brief Структура для зберігання 2D координат точки.
   */
  struct Point {
    double x;  ///< X координата (довгота або UTM easting)
    double y;  ///< Y координата (широта або UTM northing)
  };

  /**
   * @brief Callback для обробки GPS повідомлень.
   * @param msg Повідомлення з GPS координатами робота.
   */
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Завантажує полігон зони з GeoJSON файлу.
   * @return true якщо завантаження успішне, false при помилці.
   */
  bool load_geojson_zone();

  /**
   * @brief Перевіряє чи точка знаходиться всередині полігону (алгоритм ray-casting).
   * @param p Точка для перевірки.
   * @param polygon Вектор точок полігону.
   * @return true якщо точка всередині полігону.
   */
  bool point_in_polygon(const Point& p, const std::vector<Point>& polygon);

  /**
   * @brief Публікує costmap зони для Nav2.
   */
  void publish_costmap();

  /**
   * @brief Публікує полігон зони як PolygonStamped.
   */
  void publish_geofence_zone();

  /**
   * @brief Публікує маркер зони для візуалізації в Mapviz.
   */
  void publish_geofence_marker();

  // Дані зони
  std::vector<Point> zone_polygon_;      ///< Полігон в географічних координатах (lon, lat)
  std::vector<Point> zone_polygon_utm_;  ///< Полігон в UTM координатах (x, y метри)
  bool polygon_loaded_;                  ///< Прапорець успішного завантаження зони
  PJ* proj_;                             ///< PROJ об'єкт для конвертації WGS84 <-> UTM

  // ROS комунікації
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;      ///< Підписка на GPS
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zone_pub_;                ///< Публікація статусу
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;         ///< Публікація costmap
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr geofence_zone_pub_;  ///< Публікація полігону
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  ///< Публікація маркера
};

}  // namespace ros_geofence

#endif  // ZONE_CHECKER_HPP_