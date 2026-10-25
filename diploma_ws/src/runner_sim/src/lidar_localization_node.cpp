#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

// EKF state: pose in the odom frame and body-frame velocities.
constexpr int kX = 0;
constexpr int kY = 1;
constexpr int kTheta = 2;
constexpr int kVx = 3;
constexpr int kVy = 4;
constexpr int kOmega = 5;
constexpr int kStateSize = 6;

using StateVector = Eigen::Matrix<double, kStateSize, 1>;
using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Matrix2d rotation(double theta)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  Eigen::Matrix2d result;
  result << c, -s, s, c;
  return result;
}

// Relative rigid motion in 2D, ordered as (x, y, theta).
struct Motion
{
  Eigen::Vector2d translation{Eigen::Vector2d::Zero()};
  double theta{0.0};
};

struct IcpResult
{
  bool valid{false};
  Motion motion;
  // Covariance of (x, y, theta) of the estimated motion.
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
  double rms_error{std::numeric_limits<double>::infinity()};
  int correspondences{0};
  double inlier_ratio{0.0};
  int iterations{0};
};

enum class EkfMode
{
  kIcp,
  kImuIcp,
};

EkfMode parse_ekf_mode(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  if (value == "ICP") {
    return EkfMode::kIcp;
  }
  if (value == "IMU_ICP") {
    return EkfMode::kImuIcp;
  }
  throw std::runtime_error("ekf_mode must be ICP or IMU_ICP, got: " + value);
}
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
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/runner_01/sensors/imu/data");
    icp_topic_ = declare_parameter<std::string>("icp_odom_topic", "icp/odometry");
    ekf_topic_ = declare_parameter<std::string>("ekf_odom_topic", "ekf/odometry");
    ekf_mode_name_ = declare_parameter<std::string>("ekf_mode", "ICP");
    ekf_mode_ = parse_ekf_mode(ekf_mode_name_);

    fused_scan_bins_ = declare_parameter<int>("fused_scan_bins", 720);
    scan_sync_tolerance_ = declare_parameter<double>("scan_sync_tolerance", 0.15);
    body_length_ = declare_parameter<double>("body_length", 1.0);
    body_width_ = declare_parameter<double>("body_width", 0.6);
    self_filter_margin_ = declare_parameter<double>("self_filter_margin", 0.1);

    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 30);
    icp_max_correspondence_distance_ = declare_parameter<double>(
      "icp_max_correspondence_distance", 0.5);
    icp_tolerance_ = declare_parameter<double>("icp_tolerance", 1e-5);
    icp_min_correspondences_ = declare_parameter<int>("icp_min_correspondences", 50);
    icp_min_inlier_ratio_ = declare_parameter<double>("icp_min_inlier_ratio", 0.3);
    icp_huber_threshold_ = declare_parameter<double>("icp_huber_threshold", 0.05);
    icp_max_step_translation_ = declare_parameter<double>("icp_max_step_translation", 1.0);
    icp_covariance_scale_ = declare_parameter<double>("icp_covariance_scale", 10.0);
    icp_min_translation_stddev_ = declare_parameter<double>("icp_min_translation_stddev", 0.005);
    icp_min_rotation_stddev_ = declare_parameter<double>("icp_min_rotation_stddev", 0.002);
    normal_neighbor_distance_ = declare_parameter<double>("normal_neighbor_distance", 0.3);

    process_linear_acceleration_noise_ = declare_parameter<double>(
      "process_linear_acceleration_noise", 1.0);
    process_lateral_acceleration_noise_ = declare_parameter<double>(
      "process_lateral_acceleration_noise", 0.2);
    process_angular_acceleration_noise_ = declare_parameter<double>(
      "process_angular_acceleration_noise", 1.5);
    imu_angular_velocity_stddev_ = declare_parameter<double>("imu_angular_velocity_stddev", 0.02);
    // Mahalanobis gate for ICP measurements; 0 disables gating (faulty ICP is always fused).
    icp_mahalanobis_gate_ = declare_parameter<double>("icp_mahalanobis_gate", 0.0);
    initial_position_stddev_ = declare_parameter<double>("initial_position_stddev", 0.05);
    initial_heading_stddev_ = declare_parameter<double>("initial_heading_stddev", 0.02);
    initial_velocity_stddev_ = declare_parameter<double>("initial_velocity_stddev", 0.1);

    state_.setZero();
    covariance_.setZero();
    covariance_.diagonal() <<
      initial_position_stddev_ * initial_position_stddev_,
      initial_position_stddev_ * initial_position_stddev_,
      initial_heading_stddev_ * initial_heading_stddev_,
      initial_velocity_stddev_ * initial_velocity_stddev_,
      initial_velocity_stddev_ * initial_velocity_stddev_,
      initial_velocity_stddev_ * initial_velocity_stddev_;
    icp_covariance_ = covariance_.topLeftCorner<3, 3>();

    front_scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      front_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLocalizationNode::front_scan_callback, this, std::placeholders::_1));
    rear_scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      rear_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLocalizationNode::rear_scan_callback, this, std::placeholders::_1));
    if (ekf_mode_ == EkfMode::kImuIcp) {
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LidarLocalizationNode::imu_callback, this, std::placeholders::_1));
    }
    icp_publisher_ = create_publisher<nav_msgs::msg::Odometry>(icp_topic_, 10);
    ekf_publisher_ = create_publisher<nav_msgs::msg::Odometry>(ekf_topic_, 10);
    fused_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(fused_scan_topic_, 10);

    RCLCPP_INFO(get_logger(), "LiDAR fusion + ICP + EKF (mode %s) for %s: front=%s rear=%s%s%s",
      ekf_mode_ == EkfMode::kIcp ? "ICP" : "IMU_ICP", robot_name_.c_str(),
      front_scan_topic_.c_str(), rear_scan_topic_.c_str(),
      ekf_mode_ == EkfMode::kImuIcp ? " imu=" : "",
      ekf_mode_ == EkfMode::kImuIcp ? imu_topic_.c_str() : "");
  }

private:
  using Point = Eigen::Vector2d;
  using PointCloud = std::vector<Point>;

  // Uniform grid for nearest-neighbour search; cell size equals the correspondence radius,
  // so a query only has to look into the 3x3 neighbouring cells.
  struct ReferenceScan
  {
    PointCloud points;
    PointCloud normals;
    double cell_size{0.5};
    std::unordered_map<std::int64_t, std::vector<int>> cells;

    static std::int64_t key(int ix, int iy)
    {
      return (static_cast<std::int64_t>(ix) << 32) ^ static_cast<std::uint32_t>(iy);
    }

    int cell_index(double value) const
    {
      return static_cast<int>(std::floor(value / cell_size));
    }

    void build_grid()
    {
      cells.clear();
      for (std::size_t index = 0; index < points.size(); ++index) {
        cells[key(cell_index(points[index].x()), cell_index(points[index].y()))].push_back(
          static_cast<int>(index));
      }
    }

    int nearest(const Point & query, double max_distance) const
    {
      const int cx = cell_index(query.x());
      const int cy = cell_index(query.y());
      double best_distance_squared = max_distance * max_distance;
      int best_index = -1;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const auto cell = cells.find(key(cx + dx, cy + dy));
          if (cell == cells.end()) {
            continue;
          }
          for (const int index : cell->second) {
            const double distance_squared = (points[static_cast<std::size_t>(index)] - query)
              .squaredNorm();
            if (distance_squared < best_distance_squared) {
              best_distance_squared = distance_squared;
              best_index = index;
            }
          }
        }
      }
      return best_index;
    }
  };

  PointCloud scan_to_points(
    const sensor_msgs::msg::LaserScan & scan, const Point & sensor_translation,
    double sensor_yaw) const
  {
    PointCloud points;
    points.reserve(scan.ranges.size());
    const Eigen::Matrix2d sensor_rotation = rotation(sensor_yaw);
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
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

  // The LiDARs sit outside the body corners, so each one sees part of its own robot.
  // Those returns move with the robot and would pull ICP towards zero motion.
  bool is_self_hit(const Point & point) const
  {
    return std::abs(point.x()) <= 0.5 * body_length_ + self_filter_margin_ &&
           std::abs(point.y()) <= 0.5 * body_width_ + self_filter_margin_;
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
    if (is_self_hit(point)) {
      return;
    }
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

  // Points are ordered by angle, so the local surface normal is estimated from the
  // neighbouring returns (PCA over a small window). Isolated points get no normal.
  ReferenceScan make_reference(const PointCloud & points) const
  {
    ReferenceScan reference;
    reference.cell_size = icp_max_correspondence_distance_;
    const int count = static_cast<int>(points.size());
    for (int index = 0; index < count; ++index) {
      const Point & center = points[static_cast<std::size_t>(index)];
      Point mean = Point::Zero();
      std::vector<Point> neighbors;
      for (int offset = -2; offset <= 2; ++offset) {
        const Point & candidate = points[static_cast<std::size_t>((index + offset + count) % count)];
        if ((candidate - center).norm() <= normal_neighbor_distance_) {
          neighbors.push_back(candidate);
          mean += candidate;
        }
      }
      if (neighbors.size() < 3) {
        continue;
      }
      mean /= static_cast<double>(neighbors.size());
      Eigen::Matrix2d scatter = Eigen::Matrix2d::Zero();
      for (const Point & neighbor : neighbors) {
        scatter += (neighbor - mean) * (neighbor - mean).transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver;
      solver.computeDirect(scatter);
      // Reject blobs that are not line-like (both eigenvalues similar).
      if (solver.eigenvalues()[0] > 0.3 * solver.eigenvalues()[1]) {
        continue;
      }
      reference.points.push_back(center);
      reference.normals.push_back(solver.eigenvectors().col(0));
    }
    reference.build_grid();
    return reference;
  }

  // Point-to-line ICP. Finds the motion T (current robot frame expressed in the previous
  // robot frame) such that T * current ~= reference. T is therefore directly the robot
  // motion between the two scans.
  IcpResult estimate_icp(
    const PointCloud & current, const ReferenceScan & reference, const Motion & initial_guess) const
  {
    IcpResult result;
    if (current.size() < static_cast<std::size_t>(icp_min_correspondences_) ||
      reference.points.size() < static_cast<std::size_t>(icp_min_correspondences_))
    {
      return result;
    }

    double theta = initial_guess.theta;
    Point translation = initial_guess.translation;
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    double error_sum = 0.0;
    int correspondence_count = 0;

    for (int iteration = 0; iteration < icp_max_iterations_; ++iteration) {
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const Eigen::Matrix2d r = rotation(theta);
      hessian.setZero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      error_sum = 0.0;
      correspondence_count = 0;

      for (const Point & source : current) {
        const Point transformed = r * source + translation;
        const int match = reference.nearest(transformed, icp_max_correspondence_distance_);
        if (match < 0) {
          continue;
        }
        const Point & normal = reference.normals[static_cast<std::size_t>(match)];
        const double residual = normal.dot(
          transformed - reference.points[static_cast<std::size_t>(match)]);
        const Point d_rotation(-s * source.x() - c * source.y(), c * source.x() - s * source.y());
        const Eigen::Vector3d jacobian(normal.dot(d_rotation), normal.x(), normal.y());
        const double weight = std::abs(residual) <= icp_huber_threshold_ ?
          1.0 : icp_huber_threshold_ / std::abs(residual);
        hessian.noalias() += weight * jacobian * jacobian.transpose();
        gradient.noalias() += weight * jacobian * residual;
        error_sum += residual * residual;
        ++correspondence_count;
      }

      result.iterations = iteration + 1;
      if (correspondence_count < icp_min_correspondences_) {
        return result;
      }
      const Eigen::Vector3d delta =
        -(hessian + 1e-9 * Eigen::Matrix3d::Identity()).ldlt().solve(gradient);
      theta = wrap_angle(theta + delta[0]);
      translation += delta.tail<2>();
      if (delta.norm() < icp_tolerance_) {
        break;
      }
    }

    result.correspondences = correspondence_count;
    result.inlier_ratio = static_cast<double>(correspondence_count) /
      static_cast<double>(current.size());
    result.rms_error = std::sqrt(error_sum / std::max(1, correspondence_count));
    result.motion.theta = theta;
    result.motion.translation = translation;
    result.valid = result.inlier_ratio >= icp_min_inlier_ratio_ &&
      translation.norm() <= icp_max_step_translation_ && std::isfinite(theta);

    // Covariance of the Gauss-Newton solution sigma^2 * H^-1 (parameters ordered theta, x, y).
    // A degenerate geometry (e.g. a long corridor) gives a large variance along the
    // unconstrained direction. The scale and floor compensate for the well-known
    // over-confidence of this estimate.
    const double residual_variance = std::max(
      result.rms_error * result.rms_error, 0.01 * 0.01);
    const Eigen::Matrix3d parameter_covariance = icp_covariance_scale_ * residual_variance *
      (hessian + 1e-6 * Eigen::Matrix3d::Identity()).inverse();
    Eigen::Matrix3d permutation;
    permutation << 0, 1, 0,
      0, 0, 1,
      1, 0, 0;
    result.covariance = permutation * parameter_covariance * permutation.transpose();
    result.covariance(0, 0) += icp_min_translation_stddev_ * icp_min_translation_stddev_;
    result.covariance(1, 1) += icp_min_translation_stddev_ * icp_min_translation_stddev_;
    result.covariance(2, 2) += icp_min_rotation_stddev_ * icp_min_rotation_stddev_;
    return result;
  }

  // ---------------------------------------------------------------- EKF

  void predict_to(const rclcpp::Time & stamp)
  {
    if (!filter_initialized_) {
      filter_time_ = stamp;
      filter_initialized_ = true;
      return;
    }
    const double dt = (stamp - filter_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    if (dt > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Large EKF prediction step %.2f s (missing sensor data?)", dt);
    }
    filter_time_ = stamp;

    const double theta = state_[kTheta];
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double vx = state_[kVx];
    const double vy = state_[kVy];

    state_[kX] += (vx * c - vy * s) * dt;
    state_[kY] += (vx * s + vy * c) * dt;
    state_[kTheta] = wrap_angle(theta + state_[kOmega] * dt);

    StateMatrix jacobian = StateMatrix::Identity();
    jacobian(kX, kTheta) = (-vx * s - vy * c) * dt;
    jacobian(kX, kVx) = c * dt;
    jacobian(kX, kVy) = -s * dt;
    jacobian(kY, kTheta) = (vx * c - vy * s) * dt;
    jacobian(kY, kVx) = s * dt;
    jacobian(kY, kVy) = c * dt;
    jacobian(kTheta, kOmega) = dt;

    // Constant-velocity model driven by white acceleration noise.
    StateMatrix process_noise = StateMatrix::Zero();
    process_noise(kVx, kVx) =
      process_linear_acceleration_noise_ * process_linear_acceleration_noise_ * dt;
    process_noise(kVy, kVy) =
      process_lateral_acceleration_noise_ * process_lateral_acceleration_noise_ * dt;
    process_noise(kOmega, kOmega) =
      process_angular_acceleration_noise_ * process_angular_acceleration_noise_ * dt;

    covariance_ = jacobian * covariance_ * jacobian.transpose() + process_noise;
  }

  // Linear Kalman update for measurements of velocity states. Returns false when gated out.
  template<int M>
  bool update(
    const Eigen::Matrix<double, M, 1> & measurement,
    const Eigen::Matrix<double, M, kStateSize> & observation,
    const Eigen::Matrix<double, M, M> & measurement_noise,
    double mahalanobis_gate)
  {
    const Eigen::Matrix<double, M, 1> innovation = measurement - observation * state_;
    const Eigen::Matrix<double, M, M> innovation_covariance =
      observation * covariance_ * observation.transpose() + measurement_noise;
    const Eigen::Matrix<double, M, M> innovation_information = innovation_covariance.inverse();
    if (mahalanobis_gate > 0.0 &&
      innovation.dot(innovation_information * innovation) > mahalanobis_gate)
    {
      return false;
    }
    const Eigen::Matrix<double, kStateSize, M> gain =
      covariance_ * observation.transpose() * innovation_information;
    state_ += gain * innovation;
    state_[kTheta] = wrap_angle(state_[kTheta]);
    // Joseph form keeps the covariance symmetric positive definite.
    const StateMatrix identity_minus_kh = StateMatrix::Identity() - gain * observation;
    covariance_ = identity_minus_kh * covariance_ * identity_minus_kh.transpose() +
      gain * measurement_noise * gain.transpose();
    return true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const rclcpp::Time stamp(message->header.stamp);
    if (!filter_initialized_) {
      return;  // The filter starts with the first fused scan.
    }
    predict_to(stamp);
    Eigen::Matrix<double, 1, 1> measurement;
    measurement << message->angular_velocity.z;
    Eigen::Matrix<double, 1, kStateSize> observation = Eigen::Matrix<double, 1, kStateSize>::Zero();
    observation(0, kOmega) = 1.0;
    Eigen::Matrix<double, 1, 1> noise;
    noise << imu_angular_velocity_stddev_ * imu_angular_velocity_stddev_;
    update<1>(measurement, observation, noise, 0.0);
  }

  // ICP returns the relative motion over [previous scan, current scan]. It is fused as the
  // average body-frame velocity over that interval, so the EKF integrates LiDAR odometry
  // and its uncertainty (the position covariance grows without absolute measurements).
  void fuse_icp(const IcpResult & icp, double dt)
  {
    Eigen::Matrix<double, 3, 1> measurement(
      icp.motion.translation.x() / dt, icp.motion.translation.y() / dt, icp.motion.theta / dt);
    Eigen::Matrix<double, 3, kStateSize> observation = Eigen::Matrix<double, 3, kStateSize>::Zero();
    observation(0, kVx) = 1.0;
    observation(1, kVy) = 1.0;
    observation(2, kOmega) = 1.0;
    const Eigen::Matrix3d noise = icp.covariance / (dt * dt);
    if (!update<3>(measurement, observation, noise, icp_mahalanobis_gate_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ICP measurement rejected by gate");
    }
  }

  void accumulate_icp_pose(const Motion & motion, const Eigen::Matrix3d & motion_covariance)
  {
    const double c = std::cos(icp_pose_[2]);
    const double s = std::sin(icp_pose_[2]);
    const Point & t = motion.translation;
    Eigen::Matrix3d pose_jacobian = Eigen::Matrix3d::Identity();
    pose_jacobian(0, 2) = -s * t.x() - c * t.y();
    pose_jacobian(1, 2) = c * t.x() - s * t.y();
    Eigen::Matrix3d motion_jacobian = Eigen::Matrix3d::Identity();
    motion_jacobian.topLeftCorner<2, 2>() = rotation(icp_pose_[2]);

    icp_pose_[0] += c * t.x() - s * t.y();
    icp_pose_[1] += s * t.x() + c * t.y();
    icp_pose_[2] = wrap_angle(icp_pose_[2] + motion.theta);
    icp_covariance_ = pose_jacobian * icp_covariance_ * pose_jacobian.transpose() +
      motion_jacobian * motion_covariance * motion_jacobian.transpose();
  }

  void publish_odometry(
    const rclcpp::Time & stamp, const Eigen::Vector3d & pose, const Eigen::Matrix3d & covariance,
    const Eigen::Vector3d & twist, const Eigen::Matrix3d & twist_covariance,
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher)
  {
    // Indices of x, y and yaw in the 6x6 row-major ROS covariance.
    constexpr int kRosIndex[3] = {0, 1, 5};
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = robot_name_ + "/odom";
    message.child_frame_id = robot_name_ + "/base_link";
    message.pose.pose.position.x = pose[0];
    message.pose.pose.position.y = pose[1];
    message.pose.pose.orientation.z = std::sin(pose[2] * 0.5);
    message.pose.pose.orientation.w = std::cos(pose[2] * 0.5);
    message.twist.twist.linear.x = twist[0];
    message.twist.twist.linear.y = twist[1];
    message.twist.twist.angular.z = twist[2];
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        const std::size_t index = static_cast<std::size_t>(kRosIndex[row] * 6 + kRosIndex[column]);
        message.pose.covariance[index] = covariance(row, column);
        message.twist.covariance[index] = twist_covariance(row, column);
      }
    }
    publisher->publish(message);
  }

  void publish_ekf(const rclcpp::Time & stamp)
  {
    publish_odometry(stamp, state_.head<3>(), covariance_.topLeftCorner<3, 3>(),
      state_.tail<3>(), covariance_.bottomRightCorner<3, 3>(), ekf_publisher_);
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
    if (current_scan.size() < static_cast<std::size_t>(icp_min_correspondences_)) {
      return;
    }
    const rclcpp::Time stamp(message.header.stamp);
    predict_to(stamp);

    if (have_previous_scan_) {
      const double dt = (stamp - previous_scan_stamp_).seconds();
      if (dt <= 0.0) {
        return;
      }
      // Initial guess: motion predicted by the EKF velocities (gyro-aided in IMU_ICP mode).
      Motion guess;
      guess.translation = Point(state_[kVx], state_[kVy]) * dt;
      guess.theta = state_[kOmega] * dt;
      const IcpResult icp = estimate_icp(current_scan, previous_reference_, guess);
      if (icp.valid) {
        accumulate_icp_pose(icp.motion, icp.covariance);
        const Eigen::Vector3d icp_twist(
          icp.motion.translation.x() / dt, icp.motion.translation.y() / dt, icp.motion.theta / dt);
        publish_odometry(stamp, icp_pose_, icp_covariance_, icp_twist, icp.covariance / (dt * dt),
          icp_publisher_);
        fuse_icp(icp, dt);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "ICP failed (correspondences=%d, inlier ratio=%.2f); EKF is only predicting",
          icp.correspondences, icp.inlier_ratio);
      }
      RCLCPP_DEBUG(get_logger(),
        "ICP valid=%d iters=%d corr=%d inliers=%.2f rms=%.4f motion=(%.3f, %.3f, %.4f)",
        icp.valid, icp.iterations, icp.correspondences, icp.inlier_ratio, icp.rms_error,
        icp.motion.translation.x(), icp.motion.translation.y(), icp.motion.theta);
    }

    previous_reference_ = make_reference(current_scan);
    previous_scan_stamp_ = stamp;
    have_previous_scan_ = true;
    publish_ekf(stamp);
  }

  std::string robot_name_;
  std::string front_scan_topic_;
  std::string rear_scan_topic_;
  std::string fused_scan_topic_;
  std::string imu_topic_;
  std::string icp_topic_;
  std::string ekf_topic_;
  std::string ekf_mode_name_;
  EkfMode ekf_mode_{EkfMode::kIcp};
  int fused_scan_bins_;
  double scan_sync_tolerance_;
  double body_length_;
  double body_width_;
  double self_filter_margin_;
  int icp_max_iterations_;
  double icp_max_correspondence_distance_;
  double icp_tolerance_;
  int icp_min_correspondences_;
  double icp_min_inlier_ratio_;
  double icp_huber_threshold_;
  double icp_max_step_translation_;
  double icp_covariance_scale_;
  double icp_min_translation_stddev_;
  double icp_min_rotation_stddev_;
  double normal_neighbor_distance_;
  double process_linear_acceleration_noise_;
  double process_lateral_acceleration_noise_;
  double process_angular_acceleration_noise_;
  double imu_angular_velocity_stddev_;
  double icp_mahalanobis_gate_;
  double initial_position_stddev_;
  double initial_heading_stddev_;
  double initial_velocity_stddev_;
  bool filter_initialized_{false};
  bool have_previous_scan_{false};
  rclcpp::Time filter_time_;
  rclcpp::Time previous_scan_stamp_;
  ReferenceScan previous_reference_;
  StateVector state_;
  StateMatrix covariance_;
  Eigen::Vector3d icp_pose_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d icp_covariance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
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
