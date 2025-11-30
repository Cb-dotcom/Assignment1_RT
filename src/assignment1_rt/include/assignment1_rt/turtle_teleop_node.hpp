#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// Terminal helpers are implemented in the .cpp and used by main()
void configureTerminal();
void restoreTerminal();

// Node that reads keyboard input and sends finite-duration velocity commands to turtles
class TurtleTeleopNode : public rclcpp::Node
{
public:
  TurtleTeleopNode();
  void spinTeleop(); 

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  double linear_speed_;
  double angular_speed_;
  double step_distance_;
  double step_angle_;

  bool moving_;
  rclcpp::Time movement_end_time_;
  geometry_msgs::msg::Twist current_cmd_;
  std::string active_turtle_;

  void printHelp();
  void publishStatus(const std::string & event);
  bool handleKey(char c);
  void startLinearMove(double direction);
  void startAngularMove(double direction);
  void updateMotion();
  void stopMotion();
};
