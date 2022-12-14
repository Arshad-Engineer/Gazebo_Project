/**
 * @copyright Copyright (c) 2022
 * @file gazeboWalker.cpp
 * @author Arshad S. (arshad22@umd.edu)
 * @brief gazebo simulation of turtlebot3 and turtlebot gazebo world
 * @version 1.0
 * @date 2022-12-02
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Using literals to the sensor messages types for more readability
using std::placeholders::_1;
using namespace std::chrono_literals;

using LIDAR = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

// Obstacle Avoidance Class
/**
 * @brief Obstacle Avoidance class, that receives laser data and steers the
 * turtle bot
 *
 */
class ObstacleAvoidance : public rclcpp::Node {
 public:
  ObstacleAvoidance() : Node("gazeboWalker") {
    // Initialize the publisher and subscriber
    auto callback = std::bind(&ObstacleAvoidance::lidar_callback, this, _1);
    m_lidar_sub = this->create_subscription<LIDAR>("scan", 10, callback);
    pub_vel = this->create_publisher<TWIST>("cmd_vel", 10);
  }

 private:
  void lidar_callback(const LIDAR& msg) {
    if (msg.header.stamp.sec == 0) {
      return;
    }
    auto scan_data = msg.ranges;
    auto start_angle = 180;
    auto angle_range = 45;
    for (int i = start_angle; i < start_angle + angle_range; i++) {
      if (scan_data[i % 360] < 0.8) {
        execAction(0.0, 0.5);
      } else {
        execAction(0.5, 0.0);
      }
    }
  }
  void execAction(int x_vel, int z_vel) {
    auto vel = TWIST();
    vel.linear.x = x_vel;
    vel.angular.z = -z_vel;
    pub_vel->publish(vel);
  }
  // Private class members
  rclcpp::Subscription<LIDAR>::SharedPtr m_lidar_sub;
  rclcpp::Publisher<TWIST>::SharedPtr pub_vel;
  rclcpp::TimerBase::SharedPtr m_timer;
  LIDAR m_last_data;
};
// main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
