#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_model/Model.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ros2_examples {

class RobotInterfaceNode : public rclcpp::Node {
public:
  RobotInterfaceNode(
      const std::string& node_name, const std::string& subscription_topic, const std::string& publisher_topic
  );

  void init_robot_model(const std::string& name);

  std::shared_ptr<robot_model::Model> robot;
  state_representation::JointState joint_state;
  bool state_received = false;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;

private:
  void robot_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::string subscription_topic_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
}// namespace ros2_examples