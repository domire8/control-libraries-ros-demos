#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointTorques.hpp>
#include <controllers/impedance/CartesianTwistController.hpp>
#include <dynamical_systems/Linear.hpp>
#include <robot_model/Model.hpp>

using namespace state_representation;
using namespace controllers::impedance;
using namespace dynamical_systems;
using namespace robot_model;
using namespace std::chrono_literals;

namespace {
class RobotInterface {
private:
  Model robot_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  JointState joint_state_;

public:
  bool state_received = false;

  explicit RobotInterface(ros::NodeHandle* node_handle, const std::string& robot_name, const std::string& urdf_path) :
      robot_(robot_name, urdf_path) {
    this->subscriber_ = node_handle->subscribe("joint_states", 10, &RobotInterface::robot_state_callback, this);
    this->publisher_ = node_handle->advertise<std_msgs::Float64MultiArray>("velocity_controller/command", 10, false);
    this->joint_state_ = JointState(robot_name, robot_.get_joint_frames());
  }

private:
  void robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!this->state_received) {
      this->state_received = true;
    }
    this->joint_state_.set_positions(msg->position);
    this->joint_state_.set_velocities(msg->velocity);
    this->joint_state_.set_torques(msg->effort);
  }

public:
  void publish(const JointTorques& command) {
    std_msgs::Float64MultiArray msg;
    msg.data = command.to_std_vector();
    this->publisher_.publish(msg);
  }

  CartesianPose get_eef_pose() {
    return this->robot_.forward_kinematics(this->joint_state_);
  }

  Jacobian get_jacobian() {
    return this->robot_.compute_jacobian(this->joint_state_);
  }


  const std::string& get_robot_name() {
    return this->robot_.get_robot_name();
  }

  const std::vector<std::string> get_robot_frames() {
    return this->robot_.get_frames();
  }
};
}

void control_loop(RobotInterface& robot, const int& freq) {
  // set a desired target and a linear ds toward the target
  CartesianPose target(robot.get_robot_frames().back(), robot.get_robot_frames().front());
  target.set_position(.5, .0, .5);
  target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
  std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
  Linear<CartesianState> linear_ds(target, gains);


  CartesianTwistController ctrl(5, 5, .1, .1);

  ros::Rate rate(freq);
  while (ros::ok()) {
    if (robot.state_received) {
      CartesianPose eef_pose = robot.get_eef_pose();
      CartesianTwist twist = linear_ds.evaluate(robot.get_eef_pose());
      twist.clamp(0.25, 0.5);
      JointTorques command = ctrl.compute_command(twist, eef_pose, robot.get_jacobian());
      robot.publish(command);
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
  RobotInterface robot(&node_handle, robot_name, urdf_path);

  int freq = 500;
  control_loop(robot, freq);
}