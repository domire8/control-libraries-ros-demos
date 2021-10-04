#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <dynamical_systems/Linear.hpp>
#include <controllers/impedance/CartesianTwistController.hpp>

#include "RobotInterfaceNode.h"

using namespace state_representation;
using namespace std::chrono_literals;

namespace ros2_examples {
class CartesianTwistControl : public ros2_examples::RobotInterfaceNode {
private:
  std::chrono::nanoseconds dt_;

  std::shared_ptr<dynamical_systems::Linear<CartesianState>> ds_;
  std::shared_ptr<controllers::impedance::CartesianTwistController> ctrl_;

  rclcpp::TimerBase::SharedPtr timer_;

public:
  CartesianTwistControl(const std::string& node_name, const std::chrono::nanoseconds& dt) :
      RobotInterfaceNode(node_name, "joint_states", "effort_controller/command"), dt_(dt) {
    std::string robot_name;
    this->get_parameter("robot_name", robot_name);
    this->init_robot_model(robot_name);

    CartesianPose target(this->robot->get_frames().back(), this->robot->get_frames().front());
    target.set_position(.6, .0, .5);
    target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
    std::vector<double> gains = {50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    this->ds_ = std::make_shared<dynamical_systems::Linear<CartesianState>>(target, gains);

    this->ctrl_ = std::make_shared<controllers::impedance::CartesianTwistController>(100, 100, 10, 10);

    this->timer_ = this->create_wall_timer(dt_, std::bind(&CartesianTwistControl::run, this));
  }

  void run() {
    if (this->state_received && rclcpp::ok()) {
      auto eef_pose = this->robot->forward_kinematics(this->joint_state);
      CartesianTwist twist = this->ds_->evaluate(eef_pose);
      twist.clamp(0.5, 0.25);
      JointTorques
          command = this->ctrl_->compute_command(twist, eef_pose, this->robot->compute_jacobian(this->joint_state));
      this->publish(command);
    }
  }

  void publish(const JointTorques& command) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = command.to_std_vector();
    this->publisher->publish(msg);
  }
};
}// namespace ros2_examples

int main(int argc, char** argv) {
  auto dt = 4ms;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_examples::CartesianTwistControl>("cartesian_twist_control", dt));
  rclcpp::shutdown();
  return 0;
}
