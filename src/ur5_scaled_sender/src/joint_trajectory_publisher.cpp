#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class ScaledTrajectoryPublisher : public rclcpp::Node {
public:
  ScaledTrajectoryPublisher()
  : Node("scaled_trajectory_publisher")
  {
    // Declare parameters with sensible defaults
    controller_name_ = this->declare_parameter<std::string>(
      "controller_name", "scaled_joint_trajectory_controller");

    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
      }
    );

    positions_param_ = this->declare_parameter<std::vector<double>>(
      "positions", std::vector<double>{0.0, -1.57, 1.57, 0.0, 0.0, 0.0});

    time_from_start_sec_ = this->declare_parameter<double>("time_from_start_sec", 2.0);
    repeat_ = this->declare_parameter<bool>("repeat", false);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 0.5);

    const auto topic = "/" + controller_name_ + "/joint_trajectory";
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic, 10);

    RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", topic.c_str());

    // Build message once
    msg_ = build_message();

    // Delay a bit to let the controller subscribe, then publish
    if (repeat_) {
      const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ScaledTrajectoryPublisher::publish_msg, this));
    } else {
      one_shot_timer_ = this->create_wall_timer(
        1000ms, [this]() {
          publish_msg();
          one_shot_timer_->cancel();
        }
      );
    }
  }

private:
  trajectory_msgs::msg::JointTrajectory build_message() {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;

    // Use Eigen to shape and validate the positions vector
    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(positions_param_.data(), positions_param_.size());

    if (static_cast<size_t>(q.size()) != joint_names_.size()) {
      RCLCPP_WARN(this->get_logger(),
        "Parameter 'positions' length (%ld) does not match 'joint_names' length (%ld). Resizing with zeros.",
        static_cast<long>(q.size()), static_cast<long>(joint_names_.size()));
      q = Eigen::VectorXd::Zero(joint_names_.size());
    }

    // Add a tiny smoothing offset as a demo of Eigen usage
    Eigen::VectorXd offset = Eigen::VectorXd::LinSpaced(q.size(), -0.05, 0.05);
    Eigen::VectorXd q_cmd = q + offset;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.assign(q_cmd.data(), q_cmd.data() + q_cmd.size());
    p.time_from_start = rclcpp::Duration::from_seconds(std::max(0.1, time_from_start_sec_));

    jt.points.push_back(p);
    return jt;
  }

  void publish_msg() {
    msg_.header.stamp = this->now();
    pub_->publish(msg_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "JointTrajectory sent to %s (points=%zu)",
                         pub_->get_topic_name(), msg_.points.size());
  }

  // Parameters
  std::string controller_name_;
  std::vector<std::string> joint_names_;
  std::vector<double> positions_param_;
  double time_from_start_sec_ {2.0};
  bool repeat_ {false};
  double publish_rate_hz_ {0.5};

  // ROS entities
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;

  // Message cache
  trajectory_msgs::msg::JointTrajectory msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScaledTrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
