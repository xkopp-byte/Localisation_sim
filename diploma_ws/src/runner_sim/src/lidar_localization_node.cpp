#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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

Eigen::Matrix2d rotation(double theta)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  Eigen::Matrix2d result;
  result << c, -s, s, c;
  return result;
}

struct IcpResult
{
  bool valid{false};
  double theta{0.0};
  Eigen::Vector2d translation{Eigen::Vector2d::Zero()};
  double mean_error{std::numeric_limits<double>::infinity()};
};
}  // namespace

class LidarLocalizationNode : public rclcpp::Node
{
public:
  LidarLocalizationNode()
  : Node("lidar_localization_node")
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "runner_01");
    front_scan_topic_ = declare_parameter<std::string>(
      "front_scan_topic", "/runner_01_lidar_front");
    rear_scan_topic_ = declare_parameter<std::string>(
      "rear_scan_topic", "/runner_01_lidar_rear");
    fused_scan_topic_ = declare_parameter<std::string>(
      "fused_scan_topic", "/runner_01/lidar_fused");
    odom_topic_ = declare_parameter<std::string>(
      "wheel_odom_topic", "/runner_01/wheel/odometry");
    icp_topic_ = declare_parameter<std::string>("icp_odom_topic", "icp/odometry");
    ekf_topic_ = declare_parameter<std::string>("ekf_odom_topic", "ekf/odometry");
    max_scan_points_ = declare_parameter<int>("max_scan_points", 400);
    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 20);
    icp_max_correspondence_distance_ = declare_parameter<double>(
      "icp_max_correspondence_distance", 2.0);
    icp_tolerance_ = declare_parameter<double>("icp_tolerance", 1e-4);
    process_position_noise_ = declare_parameter<double>("process_position_noise", 0.02);
    process_heading_noise_ = declare_parameter<double>("process_heading_noise", 0.02);
    measurement_position_noise_ = declare_parameter<double>("measurement_position_noise", 0.08);
    measurement_heading_noise_ = declare_parameter<double>("measurement_heading_noise", 0.05);
    fused_scan_bins_ = declare_parameter<int>("fused_scan_bins", 720);
    scan_sync_tolerance_ = declare_parameter<double>("scan_sync_tolerance", 0.15);

    state_.setZero();
    covariance_.setIdentity();
    covariance_ *= 0.5;

    front_scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      front_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLocalizationNode::front_scan_callback, this, std::placeholders::_1));
    rear_scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      rear_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLocalizationNode::rear_scan_callback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&LidarLocalizationNode::odom_callback, this, std::placeholders::_1));
    icp_publisher_ = create_publisher<nav_msgs::msg::Odometry>(icp_topic_, 10);
    ekf_publisher_ = create_publisher<nav_msgs::msg::Odometry>(ekf_topic_, 10);
    fused_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(fused_scan_topic_, 10);

    RCLCPP_INFO(get_logger(), "LiDAR fusion + ICP + EKF for %s: front=%s rear=%s odom=%s",
      robot_name_.c_str(), front_scan_topic_.c_str(), rear_scan_topic_.c_str(), odom_topic_.c_str());
  }

private:
  using Point = Eigen::Vector2d;
  using PointCloud = std::vector<Point>;

  PointCloud scan_to_points(
    const sensor_msgs::msg::LaserScan & scan, const Point & sensor_translation,
    double sensor_yaw) const
  {
    PointCloud points;
    const Eigen::Matrix2d sensor_rotation = rotation(sensor_yaw);
    const std::size_t step = std::max<std::size_t>(1, scan.ranges.size() /
      static_cast<std::size_t>(std::max(1, max_scan_points_)));
    for (std::size_t index = 0; index < scan.ranges.size(); index += step) {
      const float range = scan.ranges[index];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const Point sensor_point(static_cast<double>(range) * std::cos(angle),
        static_cast<double>(range) * std::sin(angle));
      points.push_back(sensor_rotation * sensor_point + sensor_translation);
    }
    return points;
  }

  PointCloud scan_to_points(const sensor_msgs::msg::LaserScan & scan) const
  {
    return scan_to_points(scan, Point::Zero(), 0.0);
  }

  sensor_msgs::msg::LaserScan fuse_scans(
    const sensor_msgs::msg::LaserScan & front,
    const sensor_msgs::msg::LaserScan & rear,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::LaserScan fused;
    fused.header.stamp = stamp;
    fused.header.frame_id = robot_name_ + "/base_link";
    fused.angle_min = -kPi;
    fused.angle_max = kPi;
    fused.angle_increment = 2.0 * kPi / static_cast<double>(fused_scan_bins_);
    fused.time_increment = 0.0;
    fused.scan_time = std::max(front.scan_time, rear.scan_time);
    fused.range_min = std::min(front.range_min, rear.range_min);
    fused.range_max = std::min(front.range_max, rear.range_max);
    fused.ranges.assign(static_cast<std::size_t>(fused_scan_bins_),
      std::numeric_limits<float>::infinity());

    const PointCloud front_points = scan_to_points(front, Point(0.56, -0.36), 0.0);
    const PointCloud rear_points = scan_to_points(rear, Point(-0.56, 0.36), kPi);
    for (const Point & point : front_points) {
      add_point_to_fused_scan(point, fused);
    }
    for (const Point & point : rear_points) {
      add_point_to_fused_scan(point, fused);
    }
    return fused;
  }

  void add_point_to_fused_scan(const Point & point, sensor_msgs::msg::LaserScan & fused) const
  {
    const double range = point.norm();
    if (range < fused.range_min || range > fused.range_max) {
      return;
    }
    double angle = std::atan2(point.y(), point.x());
    if (angle < fused.angle_min) {
      angle += 2.0 * kPi;
    }
    const int index = static_cast<int>(std::floor(
      (angle - fused.angle_min) / fused.angle_increment));
    if (index >= 0 && index < fused_scan_bins_) {
      fused.ranges[static_cast<std::size_t>(index)] = std::min(
        fused.ranges[static_cast<std::size_t>(index)], static_cast<float>(range));
    }
  }

  IcpResult estimate_icp(const PointCloud & current, const PointCloud & reference) const
  {
    IcpResult result;
    if (current.size() < 10 || reference.size() < 10) {
      return result;
    }

    double theta = 0.0;
    Point translation = Point::Zero();
    double previous_error = std::numeric_limits<double>::infinity();

    for (int iteration = 0; iteration < icp_max_iterations_; ++iteration) {
      const Eigen::Matrix2d r = rotation(theta);
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      double error_sum = 0.0;
      int correspondence_count = 0;

      for (const Point & source : current) {
        const Point transformed = r * source + translation;
        double best_distance_squared = icp_max_correspondence_distance_ *
          icp_max_correspondence_distance_;
        Point best_match = Point::Zero();
        bool found = false;
        for (const Point & target : reference) {
          const double distance_squared = (transformed - target).squaredNorm();
          if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            best_match = target;
            found = true;
          }
        }
        if (!found) {
          continue;
        }

        const Point residual = transformed - best_match;
        Point d_rotation;
        d_rotation << -std::sin(theta) * source.x() - std::cos(theta) * source.y(),
          std::cos(theta) * source.x() - std::sin(theta) * source.y();
        Eigen::Matrix<double, 2, 3> jacobian;
        jacobian << d_rotation.x(), 1.0, 0.0,
          d_rotation.y(), 0.0, 1.0;
        hessian.noalias() += jacobian.transpose() * jacobian;
        gradient.noalias() += jacobian.transpose() * residual;
        error_sum += residual.squaredNorm();
        ++correspondence_count;
      }

      if (correspondence_count < 10) {
        return result;
      }
      const Eigen::Vector3d delta = -hessian.ldlt().solve(gradient);
      theta = wrap_angle(theta + delta[0]);
      translation += delta.tail<2>();
      const double mean_error = std::sqrt(error_sum / correspondence_count);
      if (std::abs(previous_error - mean_error) < icp_tolerance_ ||
        delta.norm() < icp_tolerance_)
      {
        result.valid = true;
        result.mean_error = mean_error;
        result.theta = theta;
        result.translation = translation;
        return result;
      }
      previous_error = mean_error;
    }

    result.valid = true;
    result.theta = theta;
    result.translation = translation;
    result.mean_error = previous_error;
    return result;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_linear_velocity_ = message->twist.twist.linear.x;
    latest_angular_velocity_ = message->twist.twist.angular.z;
    latest_odom_stamp_ = message->header.stamp;
    have_odom_ = true;
  }

  void predict(double dt)
  {
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }
    const double theta = state_[2];
    state_[0] += latest_linear_velocity_ * dt * std::cos(theta);
    state_[1] += latest_linear_velocity_ * dt * std::sin(theta);
    state_[2] = wrap_angle(state_[2] + latest_angular_velocity_ * dt);

    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
    jacobian(0, 2) = -latest_linear_velocity_ * dt * std::sin(theta);
    jacobian(1, 2) = latest_linear_velocity_ * dt * std::cos(theta);
    Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero();
    process_noise(0, 0) = process_position_noise_ * process_position_noise_;
    process_noise(1, 1) = process_position_noise_ * process_position_noise_;
    process_noise(2, 2) = process_heading_noise_ * process_heading_noise_;
    covariance_ = jacobian * covariance_ * jacobian.transpose() + process_noise;
  }

  void update(const Eigen::Vector3d & measurement)
  {
    const Eigen::Matrix3d measurement_noise = (Eigen::Vector3d(
      measurement_position_noise_ * measurement_position_noise_,
      measurement_position_noise_ * measurement_position_noise_,
      measurement_heading_noise_ * measurement_heading_noise_)).asDiagonal();
    const Eigen::Matrix3d innovation_covariance = covariance_ + measurement_noise;
    const Eigen::Matrix3d gain = covariance_ * innovation_covariance.inverse();
    Eigen::Vector3d innovation = measurement - state_;
    innovation[2] = wrap_angle(innovation[2]);
    state_ += gain * innovation;
    state_[2] = wrap_angle(state_[2]);
    covariance_ = (Eigen::Matrix3d::Identity() - gain) * covariance_;
  }

  void publish_pose(
    const rclcpp::Time & stamp, const Eigen::Vector3d & pose,
    const Eigen::Matrix3d & covariance, const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher)
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = robot_name_ + "/odom";
    message.child_frame_id = robot_name_ + "/base_link";
    message.pose.pose.position.x = pose[0];
    message.pose.pose.position.y = pose[1];
    message.pose.pose.orientation.z = std::sin(pose[2] * 0.5);
    message.pose.pose.orientation.w = std::cos(pose[2] * 0.5);
    message.pose.covariance[0] = covariance(0, 0);
    message.pose.covariance[1] = covariance(0, 1);
    message.pose.covariance[5] = covariance(0, 2);
    message.pose.covariance[6] = covariance(1, 0);
    message.pose.covariance[7] = covariance(1, 1);
    message.pose.covariance[11] = covariance(1, 2);
    message.pose.covariance[30] = covariance(2, 0);
    message.pose.covariance[31] = covariance(2, 1);
    message.pose.covariance[35] = covariance(2, 2);
    publisher->publish(message);
  }

  void rear_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    latest_rear_scan_ = message;
  }

  void front_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (!latest_rear_scan_) {
      return;
    }
    const rclcpp::Time front_stamp(message->header.stamp);
    const rclcpp::Time rear_stamp(latest_rear_scan_->header.stamp);
    if (std::abs((front_stamp - rear_stamp).seconds()) > scan_sync_tolerance_) {
      return;
    }
    const sensor_msgs::msg::LaserScan fused = fuse_scans(*message, *latest_rear_scan_, front_stamp);
    fused_scan_publisher_->publish(fused);
    process_scan(fused);
  }

  void process_scan(const sensor_msgs::msg::LaserScan & message)
  {
    const PointCloud current_scan = scan_to_points(message);
    if (current_scan.size() < 10) {
      return;
    }
    const rclcpp::Time stamp(message.header.stamp);
    if (!have_previous_scan_) {
      previous_scan_ = current_scan;
      previous_scan_stamp_ = stamp;
      have_previous_scan_ = true;
      publish_pose(stamp, state_, covariance_, icp_publisher_);
      publish_pose(stamp, state_, covariance_, ekf_publisher_);
      return;
    }

    const double dt = (stamp - previous_scan_stamp_).seconds();
    predict(dt);
    const IcpResult icp = estimate_icp(current_scan, previous_scan_);
    if (icp.valid) {
      const Eigen::Matrix2d inverse_rotation = rotation(icp.theta).transpose();
      const Point robot_translation = -inverse_rotation * icp.translation;
      const double robot_heading = -icp.theta;
      icp_pose_[0] += std::cos(icp_pose_[2]) * robot_translation.x() -
        std::sin(icp_pose_[2]) * robot_translation.y();
      icp_pose_[1] += std::sin(icp_pose_[2]) * robot_translation.x() +
        std::cos(icp_pose_[2]) * robot_translation.y();
      icp_pose_[2] = wrap_angle(icp_pose_[2] + robot_heading);
      publish_pose(stamp, icp_pose_, covariance_, icp_publisher_);
      update(icp_pose_);
      publish_pose(stamp, state_, covariance_, ekf_publisher_);
    }
    previous_scan_ = current_scan;
    previous_scan_stamp_ = stamp;
  }

  std::string robot_name_;
  std::string front_scan_topic_;
  std::string rear_scan_topic_;
  std::string fused_scan_topic_;
  std::string odom_topic_;
  std::string icp_topic_;
  std::string ekf_topic_;
  int max_scan_points_;
  int icp_max_iterations_;
  double icp_max_correspondence_distance_;
  double icp_tolerance_;
  double process_position_noise_;
  double process_heading_noise_;
  double measurement_position_noise_;
  double measurement_heading_noise_;
  int fused_scan_bins_;
  double scan_sync_tolerance_;
  double latest_linear_velocity_{0.0};
  double latest_angular_velocity_{0.0};
  bool have_odom_{false};
  bool have_previous_scan_{false};
  rclcpp::Time latest_odom_stamp_;
  rclcpp::Time previous_scan_stamp_;
  PointCloud previous_scan_;
  Eigen::Vector3d state_;
  Eigen::Vector3d icp_pose_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr icp_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_scan_publisher_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_rear_scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
