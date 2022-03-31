#include <chrono>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "RobotInterfaceNode.h"

using namespace state_representation;
using namespace std::chrono_literals;

std::function<void(int)> sigint_handler;

namespace ros2_examples {

class JointSpacePositionControl : public RobotInterfaceNode {
private:
  std::chrono::nanoseconds dt_;

  std::shared_ptr<dynamical_systems::IDynamicalSystem<CartesianState>> ds_;

  rclcpp::TimerBase::SharedPtr timer_;

public:
  JointSpacePositionControl(const std::string& node_name, const std::chrono::nanoseconds& dt) :
      RobotInterfaceNode(node_name, "joint_states", "position_controller/command"), dt_(dt) {
    std::string robot_name;
    this->get_parameter("robot_name", robot_name);
    this->init_robot_model(robot_name);

    CartesianPose target(this->robot->get_frames().back(), this->robot->get_frames().front());
    target.set_position(.4, .1, .2);
    target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
    std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    this->ds_ = dynamical_systems::DynamicalSystemFactory<CartesianState>::create_dynamical_system(
        dynamical_systems::DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
    );
    this->ds_->set_parameter_value("attractor", target);
    this->ds_->set_parameter_value("gain", std::vector<double>{50.0, 50.0, 50.0, 10.0, 10.0, 10.0});

    this->timer_ = this->create_wall_timer(dt_, [this] { control_loop(); });
  }

  void control_loop() {
    if (this->state_received && rclcpp::ok()) {
      CartesianTwist twist = this->ds_->evaluate(this->robot->forward_kinematics(this->joint_state));
      twist.clamp(0.25, 0.25);
      JointVelocities command = this->robot->inverse_velocity(twist, this->joint_state);
      this->publish(JointPositions(this->joint_state) + this->dt_ * command);
    }
  }

  void publish(const JointPositions& command) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = command.to_std_vector();
    this->publisher->publish(msg);
  }

  void stop() {
    this->publish(JointPositions(this->joint_state));
    rclcpp::shutdown();
  }
};
}// namespace ros2_examples

int main(int argc, char** argv) {
  auto dt = 4ms;
  rclcpp::init(argc, argv);
  auto control = std::make_shared<ros2_examples::JointSpacePositionControl>("joint_space_position_control", dt);
  sigint_handler = [&](int) {
    control->stop();
    exit(1);
  };
  std::signal(SIGINT, [](int signal) { sigint_handler(signal); });
  rclcpp::spin(control);
  return 0;
}
