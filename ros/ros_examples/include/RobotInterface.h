#pragma once

#include <robot_model/Model.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/joint/JointState.hpp>

namespace ros_examples {

class RobotInterface {
public:
  RobotInterface(
      ros::NodeHandle* node_handle, const std::string& publishing_topic, const std::string& robot_name,
      const std::string& urdf_path
  );

  state_representation::JointState get_joint_state();
  state_representation::CartesianPose get_eef_pose();
  state_representation::Jacobian get_jacobian();
  state_representation::JointVelocities inverse_velocity(const state_representation::CartesianTwist& twist);

  const std::string& get_robot_name();
  std::vector<std::string> get_robot_frames();

  ros::Publisher publisher;

  bool state_received = false;

private:
  void robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

  ros::Subscriber subscriber_;

  robot_model::Model robot_;
  state_representation::JointState joint_state_;
};
}// namespace ros_examples