#include "RobotInterfaceNode.h"

namespace ros2_examples {

RobotInterfaceNode::RobotInterfaceNode(
    const std::string& node_name, const std::string& subscription_topic, const std::string& publisher_topic
) : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false)), subscription_topic_(subscription_topic) {
  this->declare_parameter<std::string>("robot_name", "robot");
  this->publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(publisher_topic, 10);
}

void RobotInterfaceNode::robot_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!this->state_received) {
    this->state_received = true;
  }
  this->joint_state.set_positions(msg->position);
  this->joint_state.set_velocities(msg->velocity);
  this->joint_state.set_torques(msg->effort);
}

void RobotInterfaceNode::init_robot_model(const std::string& name) {
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
  auto urdf_string = parameters_client->get_parameter<std::string>("robot_description");

  std::string urdf_path = "/tmp/" + name + ".urdf";
  robot_model::Model::create_urdf_from_string(urdf_string, urdf_path);
  this->robot = std::make_shared<robot_model::Model>(name, urdf_path);
  this->joint_state = state_representation::JointState(this->robot->get_robot_name(), this->robot->get_joint_frames());
  this->subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      this->subscription_topic_, 10, std::bind(&RobotInterfaceNode::robot_state_callback, this, std::placeholders::_1));
}
}// namespace ros2_examples