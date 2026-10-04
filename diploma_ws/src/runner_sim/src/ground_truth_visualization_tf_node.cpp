#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class GroundTruthVisualizationTfNode : public rclcpp::Node
{
public:
  GroundTruthVisualizationTfNode()
  : Node("ground_truth_visualization_tf_node"),
    broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "runner_01");
    ground_truth_topic_ = declare_parameter<std::string>(
      "ground_truth_topic", "/runner_01/ground_truth/odometry");
    ground_truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic_, rclcpp::QoS(20),
      std::bind(
        &GroundTruthVisualizationTfNode::ground_truth_callback, this,
        std::placeholders::_1));
  }

private:
  void ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = robot_name_ + "/base_link";
    transform.transform.translation.x = message->pose.pose.position.x;
    transform.transform.translation.y = message->pose.pose.position.y;
    transform.transform.translation.z = message->pose.pose.position.z;

    const auto & orientation = message->pose.pose.orientation;
    tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    if (quaternion.length2() < 1e-12) {
      quaternion.setRPY(0.0, 0.0, 0.0);
    }
    quaternion.normalize();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    broadcaster_->sendTransform(transform);
  }

  std::string robot_name_;
  std::string ground_truth_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthVisualizationTfNode>());
  rclcpp::shutdown();
  return 0;
}
