#include "my_components/pre_approach.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace my_components
{
    PreApproach::PreApproach(const rclcpp::NodeOptions & options)
        : Node("pre_approach", options){
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PreApproach::scan_callback, this,
                  std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PreApproach::odom_callback, this,
                  std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
    }

   void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    float distance_to_obstacle = msg->ranges[msg->ranges.size() / 2];

    RCLCPP_INFO(this->get_logger(), "Distance to obstacle: %f", distance_to_obstacle);

    if (distance_to_obstacle < obstacle) 
    {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping robot.");
      obstacle_detected = true;
    } 
    else if (!obstacle_detected) 
    {
      move_forward();
    }
  }

  void  PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double s = msg->pose.pose.orientation.w;
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;

    // Calculate yaw angle from the quaternion
    current_yaw = atan2(2.0 * (y * x + s * z), s * s + x * x - y * y - z * z);

    if (obstacle_detected && !is_rotated) {
      rotate_robot();
    }
  }

  void  PreApproach::move_forward() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.5; 
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward");
  }

  void  PreApproach::rotate_robot() {

    double yaw_error = target_yaw - current_yaw;

    if (yaw_error > M_PI) {
      yaw_error -= 2 * M_PI;
    } else if (yaw_error < -M_PI) {
      yaw_error += 2 * M_PI;
    }

    RCLCPP_INFO(this->get_logger(),
                "current_yaw: %f, target_yaw: %f, yaw_error: %f", current_yaw,
                target_yaw, yaw_error);

    if (abs(yaw_error) < yaw_tolerance) {
      is_rotated = true;
      angular_speed = 0.0;
      RCLCPP_INFO(this->get_logger(),
                  "Yaw error within tolerance, rotation is complete");
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
      rclcpp::shutdown();
    } else {
      angular_speed = kp * yaw_error;
      RCLCPP_INFO(this->get_logger(), "Yaw error outside tolerance, rotating");
          geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = angular_speed;
    publisher_->publish(msg);
    }

  }
 
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
