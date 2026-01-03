/**
 * @file fake_robot.hpp
 * @brief Заголовковий файл ноди FakeRobot для тестування без реального робота.
 * 
 * Нода публікує статичну GPS позицію та TF трансформації,
 * що дозволяє тестувати ros_geofence без фізичного робота.
 */

#ifndef FAKE_ROBOT_HPP_
#define FAKE_ROBOT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ros_geofence
{

/**
 * @class FakeRobot
 * @brief ROS 2 нода що імітує робота з GPS для тестування.
 * 
 * Публікує:
 * - GPS позицію на /gps/fix
 * - TF трансформацію map -> base_link
 */
class FakeRobot : public rclcpp::Node
{
public:
  /**
   * @brief Конструктор. Ініціалізує параметри та запускає таймер публікації.
   */
  FakeRobot();

private:
  /**
   * @brief Callback таймера. Публікує GPS та TF з заданою частотою.
   */
  void timer_callback();

  // Параметри позиції робота
  double latitude_;      ///< Широта в градусах (WGS84)
  double longitude_;     ///< Довгота в градусах (WGS84)
  double altitude_;      ///< Висота в метрах
  double publish_rate_;  ///< Частота публікації в Hz

  // ROS комунікації
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;  ///< Публікатор GPS
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;      ///< Broadcaster TF
  rclcpp::TimerBase::SharedPtr timer_;                                  ///< Таймер публікації
};

}  // namespace ros_geofence

#endif  // FAKE_ROBOT_HPP_
