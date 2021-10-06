#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/robot/JointTorques.hpp>
#include <controllers/impedance/CartesianTwistController.hpp>
#include <dynamical_systems/Linear.hpp>

using namespace state_representation;
using namespace controllers::impedance;
using namespace dynamical_systems;
using namespace std::chrono_literals;

namespace {
class RobotInterface {
private:
  ros::Subscriber odom_subscriber_;
  ros::Publisher base_twist_publisher_;
  JointState joint_state_;
  CartesianState base_state_;

public:
  bool odom_received = false;

  explicit RobotInterface(ros::NodeHandle* node_handle) {
    this->base_state_ = CartesianState("robotbase_footprint", "odom");
    this->odom_subscriber_ = node_handle->subscribe("/robot/odom", 10, &RobotInterface::odom_callback, this);
    this->base_twist_publisher_ = node_handle->advertise<geometry_msgs::Twist>("/robot/base/cmd_vel", 10, false);
//    this->joint_state_ = JointState("franka", 7);
  }

private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!this->odom_received) {
      this->odom_received = true;
    }
    this->base_state_.set_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    this->base_state_.set_orientation(
        std::vector<double>{
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        }
    );
    this->base_state_.set_linear_velocity(
        Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z));
    this->base_state_.set_angular_velocity(
        Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z));
  }

//  void robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
//    if (!this->state_received) {
//      this->state_received = true;
//    }
//    this->joint_state_.set_positions(msg->position);
//    this->joint_state_.set_velocities(msg->velocity);
//    this->joint_state_.set_torques(msg->effort);
//  }

public:
  void publish_base_twist(const CartesianTwist& command) {
    geometry_msgs::Twist msg;
    msg.linear.x = command.get_linear_velocity().x();
    msg.linear.y = command.get_linear_velocity().y();
    msg.linear.z = command.get_linear_velocity().z();
    msg.angular.x = command.get_angular_velocity().x();
    msg.angular.y = command.get_angular_velocity().y();
    msg.angular.z = command.get_angular_velocity().z();
    this->base_twist_publisher_.publish(msg);
  }

  CartesianPose get_base_pose() {
    return this->base_state_;
  }
//
//  Jacobian get_jacobian() {
//    return this->robot_.compute_jacobian(this->joint_state_);
//  }
//
//  const std::string& get_robot_name() {
//    return this->robot_.get_robot_name();
//  }
//
//  const std::vector<std::string> get_robot_frames() {
//    return this->robot_.get_frames();
//  }
};
}

void control_loop(RobotInterface& robot, const int& freq) {
  // set a desired target and a linear ds toward the target for the base
  CartesianPose base_target("robotbase_footprint", "odom");
  base_target.set_position(5, 0, 0);
  Linear<CartesianState> base_ds(base_target, {50.0, 50.0, 0, 0, 0, 10.0});

  ros::Rate rate(freq);
  while (ros::ok()) {
    if (robot.odom_received) {
      CartesianTwist twist = base_ds.evaluate(robot.get_base_pose());
      twist.clamp(0.25, 0.5);
      robot.publish_base_twist(twist);
      rate.sleep();
    }
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mobile_manipulator_control");
  ros::NodeHandle node_handle;

  RobotInterface robot(&node_handle);

  int freq = 500;
  control_loop(robot, freq);
}