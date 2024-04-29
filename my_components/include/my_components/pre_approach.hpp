#ifndef COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION__PRE_APPROACH_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC                                            // Macros
      explicit PreApproach(const rclcpp::NodeOptions &options); // constructor

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float obstacle = 0.4;
  int degrees = -99;
  float yaw_tolerance = 0.05;
  float angular_speed = 0.0;
  bool obstacle_detected = false;
  bool is_rotated = false;
  double current_yaw = 0.0;
  double kp = 0.5;
  double target_yaw = degrees * M_PI / 180.0;
  bool rotate_operation = true;

  // methods
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void move_forward();
  void rotate_robot();
};

} // namespace my_components

#endif // COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
