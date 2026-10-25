#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <runner_sim/msg/mission_command.hpp>
#include <runner_sim/msg/mission_status.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}
}  // namespace

class MotionControllerNode : public rclcpp::Node
{
public:
  MotionControllerNode()
  : Node("motion_controller_node")
  {
    runner_number_ = declare_parameter<int>("runner_number", 1);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 2.0);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", kPi / 2.0);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.08);
    heading_gain_ = declare_parameter<double>("heading_gain", 2.0);
    // Above this heading error the runner turns in place; it drives again once aligned.
    rotate_in_place_threshold_ = declare_parameter<double>("rotate_in_place_threshold", 0.5);
    aligned_threshold_ = declare_parameter<double>("aligned_threshold", 0.05);
    slowdown_distance_ = declare_parameter<double>("slowdown_distance", 0.5);
    // The runner is driven from ground truth (world frame); mission targets are world
    // coordinates. Localization (EKF) runs separately and never steers the runner.
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "ground_truth/odometry");
    command_topic_ = declare_parameter<std::string>("command_topic", "mission_command");
    status_topic_ = declare_parameter<std::string>("status_topic", "mission_status");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(20),
      std::bind(&MotionControllerNode::odometry_callback, this, std::placeholders::_1));
    command_subscription_ = create_subscription<runner_sim::msg::MissionCommand>(
      command_topic_, 10,
      std::bind(&MotionControllerNode::command_callback, this, std::placeholders::_1));
    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    status_publisher_ = create_publisher<runner_sim::msg::MissionStatus>(status_topic_, 10);
  }

private:
  void command_callback(const runner_sim::msg::MissionCommand::SharedPtr message)
  {
    if (message->command_id == last_command_id_) {
      return;
    }
    pending_command_ = *message;
    has_pending_command_ = true;
    if (has_odometry_) {
      start_pending_command();
    }
  }

  void start_pending_command()
  {
    if (!has_pending_command_) {
      return;
    }
    active_command_ = pending_command_;
    last_command_id_ = active_command_.command_id;
    has_pending_command_ = false;
    active_ = true;
    reached_reported_ = false;
    rotating_in_place_ = true;

    target_x_ = active_command_.x;
    target_y_ = active_command_.y;
    circle_active_ = active_command_.circle_radius_m != 0.0;
    if (circle_active_) {
      configure_circle();
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    current_x_ = message->pose.pose.position.x;
    current_y_ = message->pose.pose.position.y;
    current_yaw_ = yaw_from_quaternion(message->pose.pose.orientation);
    has_odometry_ = true;
    if (has_pending_command_) {
      start_pending_command();
    }
    if (!active_) {
      if (active_command_.hard_point) {
        publish_zero_velocity();
      }
      return;
    }
    if (circle_active_) {
      drive_circle();
      return;
    }

    const double error_x = target_x_ - current_x_;
    const double error_y = target_y_ - current_y_;
    const double distance = std::hypot(error_x, error_y);
    if (distance <= position_tolerance_) {
      active_ = false;
      if (active_command_.hard_point) {
        publish_zero_velocity();
      }
      if (!reached_reported_) {
        publish_status(true);
        reached_reported_ = true;
      }
      return;
    }

    const double target_yaw = std::atan2(error_y, error_x);
    const double heading_error = wrap_angle(target_yaw - current_yaw_);
    const double speed_fraction = std::clamp(active_command_.speed_percent, 0.0, 100.0) / 100.0;
    if (std::abs(heading_error) > rotate_in_place_threshold_) {
      rotating_in_place_ = true;
    } else if (std::abs(heading_error) < aligned_threshold_) {
      rotating_in_place_ = false;
    }
    geometry_msgs::msg::Twist command;
    command.angular.z = std::clamp(
      heading_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
    if (!rotating_in_place_) {
      // Keep correcting the heading while driving and slow down close to the target.
      const double approach = std::clamp(distance / slowdown_distance_, 0.2, 1.0);
      command.linear.x = max_linear_speed_ * speed_fraction * approach *
        std::max(0.0, std::cos(heading_error));
    }
    velocity_publisher_->publish(command);
    publish_status(false, distance);
  }

  void configure_circle()
  {
    const double radius = active_command_.circle_radius_m;
    circle_center_x_ = current_x_ - radius * std::sin(current_yaw_);
    circle_center_y_ = current_y_ + radius * std::cos(current_yaw_);
  }

  void drive_circle()
  {
    const double radius = active_command_.circle_radius_m;
    const double distance_to_target = std::hypot(target_x_ - current_x_, target_y_ - current_y_);
    const double radius_error = std::abs(
      std::hypot(target_x_ - circle_center_x_, target_y_ - circle_center_y_) - std::abs(radius));
    if (radius_error > 0.15) {
      active_ = false;
      publish_zero_velocity();
      publish_status(false, distance_to_target, false,
        "circle endpoint is incompatible with the requested radius");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Circle command %d rejected: endpoint is incompatible with radius %.3f",
        active_command_.command_id, radius);
      return;
    }
    if (distance_to_target <= position_tolerance_) {
      active_ = false;
      if (active_command_.hard_point) {
        publish_zero_velocity();
      }
      if (!reached_reported_) {
        publish_status(true, 0.0);
        reached_reported_ = true;
      }
      return;
    }
    const double speed = max_linear_speed_ *
      std::clamp(active_command_.speed_percent, 0.0, 100.0) / 100.0;
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    command.angular.z = speed / radius;
    velocity_publisher_->publish(command);
    publish_status(false, distance_to_target);
  }

  void publish_zero_velocity()
  {
    velocity_publisher_->publish(geometry_msgs::msg::Twist());
  }

  void publish_status(
    bool reached, double distance_remaining = 0.0, bool valid = true,
    const std::string & error = "")
  {
    runner_sim::msg::MissionStatus status;
    status.command_id = active_command_.command_id;
    status.runner_number = runner_number_;
    status.active = active_;
    status.reached = reached;
    status.hard_point = active_command_.hard_point;
    status.distance_remaining = distance_remaining;
    status.valid = valid;
    status.error = error;
    status_publisher_->publish(status);
  }

  int runner_number_{1};
  double max_linear_speed_{2.0};
  double max_angular_speed_{kPi / 2.0};
  double position_tolerance_{0.08};
  double heading_gain_{2.0};
  double rotate_in_place_threshold_{0.5};
  double aligned_threshold_{0.05};
  double slowdown_distance_{0.5};
  bool rotating_in_place_{true};
  std::string odometry_topic_;
  std::string command_topic_;
  std::string status_topic_;
  std::string cmd_vel_topic_;
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  bool has_odometry_{false};
  bool has_pending_command_{false};
  bool active_{false};
  bool circle_active_{false};
  double circle_center_x_{0.0};
  double circle_center_y_{0.0};
  bool reached_reported_{false};
  int last_command_id_{-1};
  runner_sim::msg::MissionCommand pending_command_;
  runner_sim::msg::MissionCommand active_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<runner_sim::msg::MissionCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<runner_sim::msg::MissionStatus>::SharedPtr status_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
