#include <controllers/impedance/CartesianTwistController.hpp>
#include <chrono>
#include <csignal>
#include <dynamical_systems/Linear.hpp>
#include <robot_model/Model.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "RobotInterface.h"

using namespace state_representation;
using namespace controllers::impedance;
using namespace dynamical_systems;
using namespace robot_model;
using namespace std::chrono_literals;

std::function<void(int)> sigint_handler;

void publish(const ros::Publisher& publisher, const JointTorques& command) {
  std_msgs::Float64MultiArray msg;
  msg.data = command.to_std_vector();
  publisher.publish(msg);
}

void control_loop(ros_examples::RobotInterface& robot, const int& freq) {
  // set a desired target and a linear ds toward the target
  CartesianPose target(robot.get_robot_frames().back(), robot.get_robot_frames().front());
  target.set_position(.3, .4, .5);
  target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Linear<CartesianState> linear_ds(target, gains);

  CartesianTwistController ctrl(1, 1, .5, .5);

  ros::Rate rate(freq);
  while (ros::ok()) {
    if (robot.state_received) {
      CartesianPose eef_pose = robot.get_eef_pose();
      CartesianTwist twist = linear_ds.evaluate(robot.get_eef_pose());
      twist.clamp(1.5, 0.5);
      JointTorques command = ctrl.compute_command(twist, eef_pose, robot.get_jacobian());
      publish(robot.publisher, command);
      rate.sleep();
    }
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cartesian_twist_control");
  ros::NodeHandle node_handle;

  std::string robot_description;
  if (!node_handle.getParam("robot_description", robot_description)) {
    ROS_ERROR("Could load parameter 'robot_description' from parameter server.");
    return -1;
  }

  std::string robot_name = ros::this_node::getNamespace();
  std::string urdf_path = "/tmp/" + robot_name + ".urdf";
  Model::create_urdf_from_string(robot_description, urdf_path);
  ros_examples::RobotInterface robot(&node_handle, "effort_controller/command", robot_name, urdf_path);

  sigint_handler = [&](int) {
    publish(robot.publisher, JointState::Zero(robot.get_robot_name(), robot.get_joint_state().get_size()));
    ros::shutdown();
  };
  std::signal(SIGINT, [](int signal) { sigint_handler(signal); });

  int freq = 500;
  control_loop(robot, freq);
}