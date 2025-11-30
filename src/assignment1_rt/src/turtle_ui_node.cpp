/*
    The key mapping and speed scaling in this teleop node are heavily inspired by the
    standard ROS 2 teleop_twist_keyboard. The implementation done below is custom for 
    this assignment. 
    Resources: 
    https://github.com/methylDragon/teleop_twist_keyboard_cpp/tree/master/src
    https://docs.ros.org/en/jade/api/turtlesim/html/teleop__turtle__key_8cpp_source.html
*/

#include "assignment1_rt/turtle_teleop_node.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <sstream>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

using namespace std::chrono_literals;

// Store original terminal settings so we can restore them on exit.
static struct termios orig_termios;
static bool terminal_configured = false;

// Put stdin into raw, non-blocking mode
void configureTerminal()
{
  if (terminal_configured) {
    return;
  }
  if (tcgetattr(0, &orig_termios) < 0) {
    perror("tcgetattr");
    return;
  }
  struct termios raw = orig_termios;
  raw.c_lflag &= ~(ICANON | ECHO);   // turning off canonical mode and echo
  raw.c_cc[VMIN] = 0;                // non-blocking read
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &raw) < 0) {
    perror("tcsetattr");
    return;
  }
  terminal_configured = true;
}

// Restore the original terminal settings once finished
void restoreTerminal()
{
  if (!terminal_configured) {
    return;
  }
  if (tcsetattr(0, TCSANOW, &orig_termios) < 0) {
    perror("tcsetattr");
  }
  terminal_configured = false;
}

static bool kbhit()
{
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  int ret = select(1, &fds, nullptr, nullptr, &tv);
  return ret > 0 && FD_ISSET(0, &fds);
}

// Read a single character from stdin , returns 0 if nothing
static char getch()
{
  char c = 0;
  if (read(0, &c, 1) < 0) {
    return 0;
  }
  return c;
}


// Node that reads keyboard input and sends finite-duration velocity commands to turtles.
TurtleTeleopNode::TurtleTeleopNode()
: rclcpp::Node("turtle_teleop_custom"),
  linear_speed_(1.0),
  angular_speed_(1.0),
  step_distance_(1.0),
  step_angle_(1.0),
  moving_(false),
  active_turtle_("turtle1")
{
  pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle1/cmd_vel_raw", 10);
  pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle2/cmd_vel_raw", 10);

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/turtle_teleop/status", 10);

  active_pub_ = pub_turtle1_; // Deault we control turtle 1

  printHelp();
  publishStatus("init");

  RCLCPP_INFO(this->get_logger(),
    "Custom teleop started. Controlling %s. "
    "step_distance=%.2f, linear_speed=%.2f, step_angle=%.2f, angular_speed=%.2f",
    active_turtle_.c_str(),
    step_distance_, linear_speed_, step_angle_, angular_speed_);
}

void TurtleTeleopNode::spinTeleop()
{
  rclcpp::Rate rate(30.0);  // 30 Hz main loop

  while (rclcpp::ok()) {
    // Non blocking keyboard read
    if (kbhit()) {
      char c = getch();
      if (c != 0) {
        if (!handleKey(c)) {
          stopMotion();
          break;
        }
      }
    }

    // Update this but for limited amount of time
    updateMotion();

    // Keep room if ROS needs to look at internal callbacks
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}

void TurtleTeleopNode::printHelp()
{
  static const char HELP_MSG[] = R"(
Reading from the keyboard and publishing to /turtleX/cmd_vel_raw!
---------------------------------------------------------------
Movement (WASD):
  w : move forward  (fixed distance)
  s : move backward (fixed distance)
  a : rotate left   (fixed angle)
  d : rotate right  (fixed angle)

Turtle selection:
  1 : control turtle1
  2 : control turtle2

Step / speed tuning:
  + / - : increase / decrease step distance
  } / { : increase / decrease step angle

Speed scaling (teleop_twist style, 10% steps):
  q / z : increase / decrease BOTH linear and angular speed
  W / X : increase / decrease linear speed only
  E / C : increase / decrease angular speed only

Other:
  SPACE : immediate stop
  h     : show this help
  q     : quit

Safety:
  Commands are sent to /turtle{1,2}/cmd_vel_raw and filtered by distance_monitor,
  which publishes the safe commands to /turtle{1,2}/cmd_vel.
---------------------------------------------------------------
)";
  std::cout << HELP_MSG << std::endl;
}

// Publish a small status update including the last action and current settings
void TurtleTeleopNode::publishStatus(const std::string & event)
{
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << "event: " << event << "\n"
      << "active_turtle: " << active_turtle_ << "\n"
      << "linear_speed: " << linear_speed_ << "\n"
      << "angular_speed: " << angular_speed_ << "\n"
      << "step_distance: " << step_distance_ << "\n"
      << "step_angle: " << step_angle_;
  msg.data = oss.str();
  status_pub_->publish(msg);
}

// Handle a single key press -> false if we need to quit
bool TurtleTeleopNode::handleKey(char c)
{
  switch (c) {
    case 'w':
      startLinearMove(+1.0);
      break;
    case 's':
      startLinearMove(-1.0);
      break;
    case 'a':
      startAngularMove(+1.0);
      break;
    case 'd':
      startAngularMove(-1.0);
      break;

    case 'q':
      linear_speed_ *= 1.1;
      angular_speed_ *= 1.1;
      publishStatus("global_speed_up");
      break;
    case 'z':
      linear_speed_ *= 0.9;
      angular_speed_ *= 0.9;
      publishStatus("global_speed_down");
      break;

    case 'W':
      linear_speed_ *= 1.1;
      publishStatus("linear_speed_up");
      break;
    case 'X':
      linear_speed_ *= 0.9;
      publishStatus("linear_speed_down");
      break;

    case 'E':
      angular_speed_ *= 1.1;
      publishStatus("angular_speed_up");
      break;
    case 'C':
      angular_speed_ *= 0.9;
      publishStatus("angular_speed_down");
      break;

    case '+':
      step_distance_ += 0.1;
      publishStatus("step_distance_up");
      break;
    case '-':
      step_distance_ = std::max(0.1, step_distance_ - 0.1);
      publishStatus("step_distance_down");
      break;
    case '}':
      step_angle_ += 0.1;
      publishStatus("step_angle_up");
      break;
    case '{':
      step_angle_ = std::max(0.1, step_angle_ - 0.1);
      publishStatus("step_angle_down");
      break;

    case '1':
      active_turtle_ = "turtle1";
      active_pub_ = pub_turtle1_;
      publishStatus("turtle1_selected");
      break;
    case '2':
      active_turtle_ = "turtle2";
      active_pub_ = pub_turtle2_;
      publishStatus("turtle2_selected");
      break;

    case ' ':
      stopMotion();
      publishStatus("stop");
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    case 'Q':
      std::cout << "Quit requested.\n";
      return false;

    default:
      break;
  }

  return true;
}

// Start a forward/backward move for a fixed distance, it will be 1 second as requested with default but if increase/decrease it will change
void TurtleTeleopNode::startLinearMove(double direction)
{
  if (linear_speed_ <= 0.0) {
    publishStatus("linear_speed_zero_cannot_move");
    return;
  }
  if (!active_pub_) {
    publishStatus("no_active_publisher");
    return;
  }

  double duration = step_distance_ / linear_speed_;  // time to travel desired step
  current_cmd_ = geometry_msgs::msg::Twist();        // reset to zero first
  current_cmd_.linear.x = direction * linear_speed_;
  current_cmd_.angular.z = 0.0;

  movement_end_time_ = this->now() + rclcpp::Duration::from_seconds(duration);
  moving_ = true;

  publishStatus(direction > 0 ? "move_forward" : "move_backward");
}

// Start a rotation left/right for a fixed angle same logic as above
void TurtleTeleopNode::startAngularMove(double direction)
{
  if (angular_speed_ <= 0.0) {
    publishStatus("angular_speed_zero_cannot_rotate");
    return;
  }
  if (!active_pub_) {
    publishStatus("no_active_publisher");
    return;
  }

  double duration = step_angle_ / angular_speed_;    // time to rotate desired angle
  current_cmd_ = geometry_msgs::msg::Twist();        // reset to zero first
  current_cmd_.linear.x = 0.0;
  current_cmd_.angular.z = direction * angular_speed_;

  movement_end_time_ = this->now() + rclcpp::Duration::from_seconds(duration);
  moving_ = true;

  publishStatus(direction > 0 ? "rotate_left" : "rotate_right");
}

// Called every loop -> keeps publishing current_cmd_ until the planned duration expires
void TurtleTeleopNode::updateMotion()
{
  if (!moving_) {
    return;  
  }
  if (!active_pub_) {
    moving_ = false;
    return;
  }

  auto now = this->now();
  if (now < movement_end_time_) {
    // Still within the time window -> keep sending
    active_pub_->publish(current_cmd_);
  } else {
    // Time is up -> stop moving
    geometry_msgs::msg::Twist stop_msg;  
    active_pub_->publish(stop_msg);
    moving_ = false;
    publishStatus("move_complete");
  }
}

void TurtleTeleopNode::stopMotion()
{
  if (!active_pub_) {
    return;
  }
  geometry_msgs::msg::Twist stop_msg;  
  active_pub_->publish(stop_msg);
  moving_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  configureTerminal();
  std::atexit(restoreTerminal);

  auto node = std::make_shared<TurtleTeleopNode>();
  node->spinTeleop();

  rclcpp::shutdown();
  restoreTerminal();
  return 0;
}
