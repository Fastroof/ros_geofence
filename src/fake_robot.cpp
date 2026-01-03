/**
 * @file fake_robot.cpp
 * @brief Реалізація ноди FakeRobot для тестування геозони без реального робота.
 */

#include "ros_geofence/fake_robot.hpp"

namespace ros_geofence
{

FakeRobot::FakeRobot()
: Node("fake_robot")
{
  // Оголошення параметрів з значеннями за замовчуванням
  // Координати за замовчуванням - центр тестової зони
  this->declare_parameter<double>("latitude", 47.254);
  this->declare_parameter<double>("longitude", 30.44);
  this->declare_parameter<double>("altitude", 100.0);
  this->declare_parameter<double>("publish_rate", 10.0);

  // Зчитування параметрів
  this->get_parameter("latitude", latitude_);
  this->get_parameter("longitude", longitude_);
  this->get_parameter("altitude", altitude_);
  this->get_parameter("publish_rate", publish_rate_);

  // Створення публікатора GPS на стандартний топік
  gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

  // Ініціалізація TF broadcaster для публікації трансформацій
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Створення таймера для періодичної публікації
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&FakeRobot::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Fake robot started at lat: %f, lon: %f", latitude_, longitude_);
}

void FakeRobot::timer_callback()
{
  auto now = this->get_clock()->now();

  // Формування та публікація GPS повідомлення
  sensor_msgs::msg::NavSatFix gps_msg;
  gps_msg.header.stamp = now;
  gps_msg.header.frame_id = "gps_link";
  gps_msg.latitude = latitude_;
  gps_msg.longitude = longitude_;
  gps_msg.altitude = altitude_;
  // Статус: маємо GPS fix
  gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  gps_pub_->publish(gps_msg);

  // Публікація статичної TF трансформації map -> base_link
  // Робот знаходиться в центрі map frame
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  // Нульова позиція та орієнтація (одинична кватерніон)
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace ros_geofence

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_geofence::FakeRobot>());
  rclcpp::shutdown();
  return 0;
}
