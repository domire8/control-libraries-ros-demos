#include <controllers/ControllerFactory.hpp>
#include <chrono>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "RobotInterfaceNode.h"

using namespace state_representation;
using namespace std::chrono_literals;

std::function<void(int)> sigint_handler;

namespace ros2_examples {
class CartesianTwistControl : public ros2_examples::RobotInterfaceNode {
private:
  std::chrono::nanoseconds dt_;

  std::shared_ptr<dynamical_systems::IDynamicalSystem<CartesianState>> ds_;
  std::shared_ptr<controllers::IController<CartesianState>> ctrl_;

  rclcpp::TimerBase::SharedPtr timer_;

public:
  CartesianTwistControl(const std::string& node_name, const std::chrono::nanoseconds& dt) :
      RobotInterfaceNode(node_name, "joint_states", "effort_controller/command"), dt_(dt) {
    std::string robot_name;
    this->get_parameter("robot_name", robot_name);
    this->init_robot_model(robot_name);

    CartesianPose target(this->robot->get_frames().back(), this->robot->get_frames().front());
    target.set_position(.3, .4, .5);
    target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
    this->ds_ = dynamical_systems::DynamicalSystemFactory<CartesianState>::create_dynamical_system(
        dynamical_systems::DynamicalSystemFactory<CartesianState>::DYNAMICAL_SYSTEM::POINT_ATTRACTOR
    );
    this->ds_->set_parameter_value("attractor", target);
    this->ds_->set_parameter_value("gain", std::vector<double>{50.0, 50.0, 50.0, 10.0, 10.0, 10.0});

    this->ctrl_ =
        controllers::CartesianControllerFactory::create_controller(controllers::CONTROLLER_TYPE::COMPLIANT_TWIST, *this->robot);
    this->ctrl_->set_parameter_value("linear_principle_damping", 1.);
    this->ctrl_->set_parameter_value("linear_orthogonal_damping", 1.);
    this->ctrl_->set_parameter_value("angular_stiffness", .5);
    this->ctrl_->set_parameter_value("angular_damping", .5);

    this->timer_ = this->create_wall_timer(dt_, [this] { run(); });
  }

  void run() {
    if (this->state_received && rclcpp::ok()) {
      auto eef_pose = this->robot->forward_kinematics(this->joint_state);
      CartesianTwist twist = this->ds_->evaluate(eef_pose);
      twist.clamp(1.5, 0.5);
      JointTorques
          command = this->ctrl_->compute_command(twist, eef_pose, this->joint_state);
      this->publish(command);
    }
  }

  void publish(const JointTorques& command) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = command.to_std_vector();
    this->publisher->publish(msg);
  }

  void stop() {
    this->publish(JointTorques::Zero(this->robot->get_robot_name(), this->robot->get_number_of_joints()));
    rclcpp::shutdown();
  }
};
}// namespace ros2_examples

int main(int argc, char** argv) {
  auto dt = 4ms;
  rclcpp::init(argc, argv);
  auto control = std::make_shared<ros2_examples::CartesianTwistControl>("cartesian_twist_control", dt);
  sigint_handler = [&](int) {
    control->stop();
    exit(1);
  };
  std::signal(SIGINT, [](int signal) { sigint_handler(signal); });
  rclcpp::spin(control);
  return 0;
}
