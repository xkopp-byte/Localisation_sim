#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class LocalizationVisualizationNode : public rclcpp::Node
{
public:
  LocalizationVisualizationNode()
  : Node("localization_visualization_node")
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "runner_01");
    ground_truth_topic_ = declare_parameter<std::string>(
      "ground_truth_topic", "/runner_01/ground_truth/odometry");
    estimate_topic_ = declare_parameter<std::string>(
      "estimate_topic", "/runner_01/ekf/odometry");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/runner_01/localization_markers");
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    show_covariance_ellipse_ = declare_parameter<bool>("show_covariance_ellipse", false);
    // Probability mass inside the drawn position ellipse (2D Gaussian).
    ellipse_confidence_ = std::clamp(
      declare_parameter<double>("covariance_ellipse_confidence", 0.95), 0.01, 0.9999);

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    ground_truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic_, rclcpp::QoS(20),
      std::bind(
        &LocalizationVisualizationNode::ground_truth_callback, this,
        std::placeholders::_1));
    estimate_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      estimate_topic_, rclcpp::QoS(20),
      std::bind(
        &LocalizationVisualizationNode::estimate_callback, this,
        std::placeholders::_1));
  }

private:
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
  {
    return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  }

  visualization_msgs::msg::Marker make_outline(
    int id, const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Time & stamp, double x, double y, double yaw) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = robot_name_ + "/localization_outline";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.orientation.z = std::sin(yaw * 0.5);
    marker.pose.orientation.w = std::cos(yaw * 0.5);
    marker.scale.x = 0.035;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    const double half_length = 0.5;
    const double half_width = 0.3;
    marker.points = {
      make_point(-half_length, -half_width, 0.03),
      make_point(half_length, -half_width, 0.03),
      make_point(half_length, half_width, 0.03),
      make_point(-half_length, half_width, 0.03),
      make_point(-half_length, -half_width, 0.03),
    };
    return marker;
  }

  visualization_msgs::msg::Marker make_center(
    int id, const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Time & stamp, double x, double y) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = robot_name_ + "/localization_center";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.04;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  // Position uncertainty ellipse from the 2x2 xy covariance (already in the world frame).
  // For a 2D Gaussian the confidence region p is the ellipse with Mahalanobis radius
  // k = sqrt(-2 ln(1 - p)); its semi-axes are k * sqrt(eigenvalues).
  visualization_msgs::msg::Marker make_covariance_ellipse(
    int id, const rclcpp::Time & stamp, double x, double y) const
  {
    const double a = covariance_xx_;
    const double b = covariance_xy_;
    const double c = covariance_yy_;
    const double mean = 0.5 * (a + c);
    const double radius = std::hypot(0.5 * (a - c), b);
    const double major_variance = std::max(mean + radius, 0.0);
    const double minor_variance = std::max(mean - radius, 0.0);
    const double major_angle = 0.5 * std::atan2(2.0 * b, a - c);
    const double scale = std::sqrt(-2.0 * std::log(1.0 - ellipse_confidence_));

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "world";
    marker.ns = robot_name_ + "/covariance_ellipse";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.01;
    marker.pose.orientation.z = std::sin(major_angle * 0.5);
    marker.pose.orientation.w = std::cos(major_angle * 0.5);
    marker.scale.x = std::max(2.0 * scale * std::sqrt(major_variance), 0.01);
    marker.scale.y = std::max(2.0 * scale * std::sqrt(minor_variance), 0.01);
    marker.scale.z = 0.01;
    marker.color = make_color(1.0F, 0.55F, 0.0F, 0.35F);
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  static geometry_msgs::msg::Point make_point(double x, double y, double z)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  void publish_markers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;
    const std_msgs::msg::ColorRGBA green = make_color(0.1F, 1.0F, 0.1F, 1.0F);
    const std_msgs::msg::ColorRGBA red = make_color(1.0F, 0.1F, 0.1F, 1.0F);
    if (have_ground_truth_) {
      markers.markers.push_back(make_outline(0, green, stamp, ground_truth_x_, ground_truth_y_, ground_truth_yaw_));
      markers.markers.push_back(make_center(1, green, stamp, ground_truth_x_, ground_truth_y_));
    }
    if (have_estimate_) {
      markers.markers.push_back(make_outline(2, red, stamp, estimate_x_, estimate_y_, estimate_yaw_));
      markers.markers.push_back(make_center(3, red, stamp, estimate_x_, estimate_y_));
      if (show_covariance_ellipse_) {
        markers.markers.push_back(make_covariance_ellipse(4, stamp, estimate_x_, estimate_y_));
      }
    }
    marker_publisher_->publish(markers);
  }

  void ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    ground_truth_x_ = message->pose.pose.position.x;
    ground_truth_y_ = message->pose.pose.position.y;
    ground_truth_yaw_ = yaw_from_quaternion(message->pose.pose.orientation);
    have_ground_truth_ = true;
    publish_markers(rclcpp::Time(message->header.stamp));
  }

  void estimate_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double cos_initial = std::cos(initial_yaw_);
    const double sin_initial = std::sin(initial_yaw_);
    const double local_x = message->pose.pose.position.x;
    const double local_y = message->pose.pose.position.y;
    estimate_x_ = initial_x_ + cos_initial * local_x - sin_initial * local_y;
    estimate_y_ = initial_y_ + sin_initial * local_x + cos_initial * local_y;
    estimate_yaw_ = initial_yaw_ + yaw_from_quaternion(message->pose.pose.orientation);
    // Rotate the odom-frame xy covariance into the world frame: R * P * R^T.
    const double pxx = message->pose.covariance[0];
    const double pxy = message->pose.covariance[1];
    const double pyy = message->pose.covariance[7];
    covariance_xx_ = cos_initial * cos_initial * pxx - 2.0 * cos_initial * sin_initial * pxy +
      sin_initial * sin_initial * pyy;
    covariance_xy_ = cos_initial * sin_initial * (pxx - pyy) +
      (cos_initial * cos_initial - sin_initial * sin_initial) * pxy;
    covariance_yy_ = sin_initial * sin_initial * pxx + 2.0 * cos_initial * sin_initial * pxy +
      cos_initial * cos_initial * pyy;
    have_estimate_ = true;
    publish_markers(rclcpp::Time(message->header.stamp));
  }

  static std_msgs::msg::ColorRGBA make_color(float red, float green, float blue, float alpha)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = red;
    color.g = green;
    color.b = blue;
    color.a = alpha;
    return color;
  }

  std::string robot_name_;
  std::string ground_truth_topic_;
  std::string estimate_topic_;
  std::string marker_topic_;
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  double ground_truth_x_{0.0};
  double ground_truth_y_{0.0};
  double ground_truth_yaw_{0.0};
  double estimate_x_{0.0};
  double estimate_y_{0.0};
  double estimate_yaw_{0.0};
  bool show_covariance_ellipse_{false};
  double ellipse_confidence_{0.95};
  double covariance_xx_{0.0};
  double covariance_xy_{0.0};
  double covariance_yy_{0.0};
  bool have_ground_truth_{false};
  bool have_estimate_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimate_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
