#include "assignment1_rt/distance_monitor_node.hpp"

using std::placeholders::_1;

// This node acts as safety check between teleop (/cmd_vel_raw) and turtlesim (/cmd_vel)
DistanceMonitorNode::DistanceMonitorNode()
: rclcpp::Node("distance_monitor")  // Node name important to match with our yaml
{
  // Declare parameters with defaults, they are overriden by our yaml config file 
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.2);
  min_coord_ = this->declare_parameter<double>("min_coord", 1.0);
  max_coord_ = this->declare_parameter<double>("max_coord", 10.0);

  RCLCPP_INFO(
    get_logger(),
    "DistanceMonitorNode started. distance_threshold=%.2f, bounds=[%.2f, %.2f]",
    distance_threshold_, min_coord_, max_coord_);

  sub_turtle1_pose_ = this->create_subscription<turtlesim::msg::Pose>(
    "/turtle1/pose", 10,
    std::bind(&DistanceMonitorNode::pose1Callback, this, _1));
  sub_turtle2_pose_ = this->create_subscription<turtlesim::msg::Pose>(
    "/turtle2/pose", 10,
    std::bind(&DistanceMonitorNode::pose2Callback, this, _1));

  sub_turtle1_cmd_raw_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/turtle1/cmd_vel_raw", 10,
    std::bind(&DistanceMonitorNode::cmd1RawCallback, this, _1));
  sub_turtle2_cmd_raw_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/turtle2/cmd_vel_raw", 10,
    std::bind(&DistanceMonitorNode::cmd2RawCallback, this, _1));

  distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/turtles_distance", 10);

  pub_turtle1_cmd_safe_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle1/cmd_vel", 10);
  pub_turtle2_cmd_safe_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle2/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), // Timer is at 50 Hz 
    std::bind(&DistanceMonitorNode::timerCallback, this));
}

void DistanceMonitorNode::pose1Callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  pose1_ = *msg;
  have_pose1_ = true;
}

void DistanceMonitorNode::pose2Callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  pose2_ = *msg;
  have_pose2_ = true;
}

void DistanceMonitorNode::cmd1RawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd1_raw_ = *msg;
  have_cmd1_raw_ = true;
}

void DistanceMonitorNode::cmd2RawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd2_raw_ = *msg;
  have_cmd2_raw_ = true;
}

// Simple Euclidean distance between the two turtles, order doesn't matter obv
double DistanceMonitorNode::distanceBetween(
  const turtlesim::msg::Pose & a,
  const turtlesim::msg::Pose & b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// Squared distance outside the allowed box -> zero if the pose is inside.
double DistanceMonitorNode::boundaryError(double x, double y) const
{
  double dx_out = 0.0;
  if (x < min_coord_) dx_out = min_coord_ - x;
  else if (x > max_coord_) dx_out = x - max_coord_;

  double dy_out = 0.0;
  if (y < min_coord_) dy_out = min_coord_ - y;
  else if (y > max_coord_) dy_out = y - max_coord_;

  return dx_out * dx_out + dy_out * dy_out;
}

// Helper to check if pose is outside the bounding box 
bool DistanceMonitorNode::violatesBoundary(const turtlesim::msg::Pose & p) const
{
  return p.x < min_coord_ || p.x > max_coord_ || p.y < min_coord_ || p.y > max_coord_;
}

// Predict where the turtle will be after applying a command after some dt (simplified 1 step Euler)
turtlesim::msg::Pose DistanceMonitorNode::predictPose(
  const turtlesim::msg::Pose & p,
  const geometry_msgs::msg::Twist & cmd,
  double dt) const
{
  turtlesim::msg::Pose out = p;
  double v = cmd.linear.x;
  double w = cmd.angular.z;

  // Simple unicycle model -> move forward in heading direction and rotate.
  out.x += v * std::cos(p.theta) * dt;
  out.y += v * std::sin(p.theta) * dt;
  out.theta += w * dt;
  return out;
}

// Return true if this command makes the turtle move away wrt the other turtle
bool DistanceMonitorNode::movesAwayFromOther(
  const turtlesim::msg::Pose & self,
  const turtlesim::msg::Pose & other,
  const geometry_msgs::msg::Twist & cmd) const
{
  double dt = 0.1;  // Short lookahead time
  double dist_now = distanceBetween(self, other);
  auto predicted = predictPose(self, cmd, dt);
  double dist_future = distanceBetween(predicted, other);

  return dist_future >= dist_now - 1e-4;
}

// Return true if this command keeps the boundary violation from getting worse
bool DistanceMonitorNode::improvesBoundary(
  const turtlesim::msg::Pose & p,
  const geometry_msgs::msg::Twist & cmd) const
{
  double e_now = boundaryError(p.x, p.y);
  if (e_now <= 0.0) { 
    return true; // Already inside box -> safe
  }

  double dt = 0.1;
  auto predicted = predictPose(p, cmd, dt);
  double e_future = boundaryError(predicted.x, predicted.y);
  return e_future <= e_now + 1e-4; // Accept command that reduces the error only
}

// Main loop
void DistanceMonitorNode::timerCallback()
{
  geometry_msgs::msg::Twist cmd1_safe; 
  geometry_msgs::msg::Twist cmd2_safe;

  // In case both of poses not available can't enforce boundaries, let pass any existing raw command
  if (!have_pose1_ || !have_pose2_) {
    if (have_cmd1_raw_) cmd1_safe = last_cmd1_raw_;
    if (have_cmd2_raw_) cmd2_safe = last_cmd2_raw_;
    pub_turtle1_cmd_safe_->publish(cmd1_safe);
    pub_turtle2_cmd_safe_->publish(cmd2_safe);
    return;
  }

  // Here we are sure we have, its safe to compute the distance
  double dist = distanceBetween(pose1_, pose2_);

  // Publishing the current distance 
  std_msgs::msg::Float32 dist_msg;
  dist_msg.data = static_cast<float>(dist);
  distance_pub_->publish(dist_msg);

  // Check for all our conditions independently and update our bools in case any validation happened 
  bool dist_violation = dist < distance_threshold_;
  bool b1_violation = violatesBoundary(pose1_);
  bool b2_violation = violatesBoundary(pose2_);
  bool any_violation = dist_violation || b1_violation || b2_violation;

  // Log only when safety state changes exclusilvy not to spam too much
  if (any_violation && !last_any_violation_) {
    RCLCPP_WARN(get_logger(),
      "Safety condition active: dist_violation=%d, b1=%d, b2=%d (dist=%.3f)",
      dist_violation, b1_violation, b2_violation, dist);
  } else if (!any_violation && last_any_violation_) {
    RCLCPP_INFO(get_logger(), "Safety condition cleared.");
  }
  last_any_violation_ = any_violation;

  // Assign our last raw command that was sent, check if it is allowed (imporves our situation) -> if not its blocked
  if (have_cmd1_raw_) {
    cmd1_safe = last_cmd1_raw_;

    // If too close to other turtle and command moves closer
    if (dist_violation && !movesAwayFromOther(pose1_, pose2_, last_cmd1_raw_)) {
      cmd1_safe = geometry_msgs::msg::Twist();
    }

    // If outside boundary and command pushes further out
    if (b1_violation && !improvesBoundary(pose1_, last_cmd1_raw_)) {
      cmd1_safe = geometry_msgs::msg::Twist();
    }
  }

  // Same idea for the second
  if (have_cmd2_raw_) {
    cmd2_safe = last_cmd2_raw_;

    if (dist_violation && !movesAwayFromOther(pose2_, pose1_, last_cmd2_raw_)) {
      cmd2_safe = geometry_msgs::msg::Twist();
    }

    if (b2_violation && !improvesBoundary(pose2_, last_cmd2_raw_)) {
      cmd2_safe = geometry_msgs::msg::Twist();
    }
  }

  // Finally, publish the safe commands for turtlesim to execute.
  pub_turtle1_cmd_safe_->publish(cmd1_safe);
  pub_turtle2_cmd_safe_->publish(cmd2_safe);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DistanceMonitorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
