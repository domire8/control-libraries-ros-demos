#include "RobotInterface.h"

#include <std_msgs/Float64MultiArray.h>

namespace ros_examples {

RobotInterface::RobotInterface(
    ros::NodeHandle* node_handle, const std::string& publishing_topic, const std::string& robot_name,
    const std::string& urdf_path
) : robot_(robot_name, urdf_path) {
  this->subscriber_ = node_handle->subscribe("joint_states", 10, &RobotInterface::robot_state_callback, this);
  this->publisher = node_handle->advertise<std_msgs::Float64MultiArray>(publishing_topic, 10, false);
  this->joint_state_ = state_representation::JointState(robot_name, robot_.get_joint_frames());
}

void RobotInterface::robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!this->state_received) {
    this->state_received = true;
  }
  this->joint_state_.set_positions(msg->position);
  this->joint_state_.set_velocities(msg->velocity);
  this->joint_state_.set_torques(msg->effort);
}

const std::string& RobotInterface::get_robot_name() {
  return this->robot_.get_robot_name();
}

std::vector<std::string> RobotInterface::get_robot_frames() {
  return this->robot_.get_frames();
}

state_representation::JointState RobotInterface::get_joint_state() {
  return this->joint_state_;
}

state_representation::CartesianPose RobotInterface::get_eef_pose() {
  return this->robot_.forward_kinematics(this->joint_state_);
}

state_representation::Jacobian RobotInterface::get_jacobian() {
  return this->robot_.compute_jacobian(this->joint_state_);
}

state_representation::JointVelocities
RobotInterface::inverse_velocity(const state_representation::CartesianTwist& twist) {
  return this->robot_.inverse_velocity(twist, this->joint_state_);
}
}// namespace ros_examples