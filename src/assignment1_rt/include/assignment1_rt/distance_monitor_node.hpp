#pragma once

#include <cmath>
#include <string>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "turtlesim/msg/pose.hpp"

// This node acts as safety check between teleop (/cmd_vel_raw) and turtlesim (/cmd_vel)
class DistanceMonitorNode : public rclcpp::Node
{
public:
  DistanceMonitorNode();

private:
  double distance_threshold_;
  double min_coord_;
  double max_coord_;

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1_pose_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle2_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_turtle1_cmd_raw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_turtle2_cmd_raw_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_cmd_safe_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_cmd_safe_;

  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose pose1_;
  turtlesim::msg::Pose pose2_;
  bool have_pose1_{false};
  bool have_pose2_{false};

  geometry_msgs::msg::Twist last_cmd1_raw_;
  geometry_msgs::msg::Twist last_cmd2_raw_;
  bool have_cmd1_raw_{false};
  bool have_cmd2_raw_{false};

  bool last_any_violation_{false};

  void pose1Callback(const turtlesim::msg::Pose::SharedPtr msg);
  void pose2Callback(const turtlesim::msg::Pose::SharedPtr msg);
  void cmd1RawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmd2RawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  double distanceBetween(const turtlesim::msg::Pose & a,
                         const turtlesim::msg::Pose & b) const;

  double boundaryError(double x, double y) const;
  bool violatesBoundary(const turtlesim::msg::Pose & p) const;

  turtlesim::msg::Pose predictPose(const turtlesim::msg::Pose & p,
                                   const geometry_msgs::msg::Twist & cmd,
                                   double dt) const;

  bool movesAwayFromOther(const turtlesim::msg::Pose & self,
                          const turtlesim::msg::Pose & other,
                          const geometry_msgs::msg::Twist & cmd) const;

  bool improvesBoundary(const turtlesim::msg::Pose & p,
                        const geometry_msgs::msg::Twist & cmd) const;

  void timerCallback();
};
