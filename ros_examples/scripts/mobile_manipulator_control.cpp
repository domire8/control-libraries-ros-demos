#include <chrono>
#include <controllers/impedance/CartesianTwistController.hpp>
#include <dynamical_systems/Linear.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <robot_model/Model.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>

using namespace state_representation;
using namespace controllers::impedance;
using namespace dynamical_systems;
using namespace robot_model;
using namespace std::chrono_literals;

namespace {

static CartesianPose from_tf_message(const geometry_msgs::TransformStamped& tf) {
  auto transform = CartesianPose(tf.child_frame_id, tf.header.frame_id);
  transform.set_position(
      tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
  );
  transform.set_orientation(
      std::vector<double>{
          tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z
      }
  );
  return transform;
}

enum STATE {
  APPROACH_1, PICK_BALL, APPROACH_2, APPROACH_3, RELEASE, GO_BACK
};

class RobotInterface {
private:
  ros::NodeHandle* nh_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher base_twist_publisher_;
  ros::Publisher arm_velocity_publisher_;
  tf2_ros::TransformListener tf_listener_;

  Model robot_;
  JointState arm_joint_state_;
  CartesianState base_state_;
  CartesianPose base_to_arm_;

public:
  tf2_ros::Buffer tf_buffer;
  bool odom_received = false;
  bool joint_state_received = false;

  explicit RobotInterface(ros::NodeHandle* node_handle, const std::string& urdf_path) :
      nh_(node_handle), tf_listener_(tf_buffer), robot_("franka", urdf_path) {
    this->base_state_ = CartesianState("robotbase_footprint", "odom");
    this->odom_subscriber_ = node_handle->subscribe("/robot/odom", 10, &RobotInterface::odom_callback, this);
    this->joint_state_subscriber_ =
        node_handle->subscribe("/robot/joint_states", 10, &RobotInterface::joint_state_callback, this);
    this->base_twist_publisher_ = node_handle->advertise<geometry_msgs::Twist>("/robot/base/cmd_vel", 10, false);
    this->arm_velocity_publisher_ =
        node_handle->advertise<std_msgs::Float64MultiArray>("/robot/arm/velocity_controller/command", 10, false);
    this->arm_joint_state_ = JointState("franka", 7);
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

  void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!this->joint_state_received) {
      this->joint_state_received = true;
    }
    std::vector<std::string> name;
    std::vector<double> position, velocity, effort;
    for (std::size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name.at(j).find("panda_") != std::string::npos) {
        name.push_back(msg->name.at(j));
        position.push_back(msg->position.at(j));
        velocity.push_back(msg->velocity.at(j));
        effort.push_back(msg->effort.at(j));
      }
    }
    this->arm_joint_state_.set_names(name);
    this->arm_joint_state_.set_positions(position);
    this->arm_joint_state_.set_velocities(velocity);
    this->arm_joint_state_.set_torques(effort);
  }

  bool get_transform(CartesianPose& transform, const std::string& target, const std::string& source) {
    geometry_msgs::TransformStamped tf;
    try {
      tf = this->tf_buffer.lookupTransform(target, source, ros::Time(0));
    } catch (tf2::TransformException& ex) {
      return false;
    }
    transform = from_tf_message(tf);
    return true;
  }

  bool service_client(const std::string& name) {
    ros::ServiceClient client = this->nh_->serviceClient<std_srvs::Trigger>(name);
    std_srvs::Trigger srv;
    if (client.call(srv)) {
      if (srv.response.success) {
        return true;
      }
      ROS_ERROR_STREAM(srv.response.message);
      return false;
    }
    return false;
  }

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

  void publish_arm_command(const JointTorques& command) {
    std_msgs::Float64MultiArray msg;
    msg.data = command.to_std_vector();
    this->arm_velocity_publisher_.publish(msg);
  }

  bool get_base_to_arm_transform(CartesianPose& transform) {
    return this->get_transform(transform, "panda_link0", "robotbase_footprint");
  }

  CartesianPose get_base_pose() {
    return this->base_state_;
  }

  CartesianPose get_eef_pose() {
    auto pose = this->robot_.forward_kinematics(this->arm_joint_state_);
    pose.set_reference_frame("panda_link0");
    return pose;
  }

  Jacobian get_jacobian() {
    auto jac = this->robot_.compute_jacobian(this->arm_joint_state_);
    return Jacobian(jac.get_name(), jac.get_frame(), jac.data(), "panda_link0");
  }

  JointVelocities inverse_velocity(const CartesianTwist& twist) {
    return robot_.inverse_velocity(twist, this->arm_joint_state_);
  }

  bool pick_ball() {
    return this->service_client("/ball/attach");
  }

  bool release() {
    return this->service_client("/ball/let_go");
  }

  bool lock_joints() {
    return this->service_client("/arm/lock_joints");
  }

};
}

void control_loop(RobotInterface& robot, const int& freq) {
  ros::Rate rate(freq);
  CartesianPose base_to_arm_base;
  while (!robot.get_base_to_arm_transform(base_to_arm_base)) {
    rate.sleep();
  }

  CartesianTwistController ctrl(5, 5, .5, .5);

  // set a desired target and a linear ds toward the target for the base
  CartesianPose base_target("robotbase_footprint", "odom");
  base_target.set_position(-1.3, 0, 0);
  Linear<CartesianState> base_ds(base_target, {50.0, 50.0, 0, 0, 0, 10.0});
  double base_max_vel = 0.4;

  // set a desired target and a linear ds toward the target for the arm
  CartesianPose arm_target("panda_link8", "panda_link0");
  arm_target.set_position(0.5, 0, 0.5);
  arm_target.set_orientation(Eigen::Quaterniond(0, 1, 0, 0));
  Linear<CartesianState> arm_ds(arm_target, {50.0, 50.0, 50.0, 10.0, 10.0, 10.0});
  double arm_max_lin = 0.4;
  double arm_max_ang = 1;

  int wait = 0;
  bool received = false;
  STATE state = STATE::APPROACH_1;
  while (ros::ok()) {
    if (robot.odom_received && robot.joint_state_received) {
      switch (state) {
        case STATE::APPROACH_1 : {
          geometry_msgs::TransformStamped tf_msg;
          try {
            tf_msg = robot.tf_buffer.lookupTransform("panda_link0", "ball", ros::Time(0));
            received = true;
          } catch (tf2::TransformException& ex) {
            received = false;
          }
          if (received) {
            auto ball_in_arm_base = from_tf_message(tf_msg);
            ball_in_arm_base.set_orientation(arm_ds.get_attractor().get_orientation());
            if (robot.get_eef_pose().dist(ball_in_arm_base, CartesianStateVariable::POSITION) < 0.07
                && robot.get_base_pose().dist(base_ds.get_attractor(), CartesianStateVariable::POSITION) < 0.01) {
              if (wait < 1000) {
                ++wait;
              } else {
                wait = 0;
                received = false;
                state = STATE::PICK_BALL;
              }
            }
            if (ball_in_arm_base.get_position().norm() > 1) {
              ball_in_arm_base.set_position(ball_in_arm_base.get_position().normalized());
            }
            ball_in_arm_base.set_position(ball_in_arm_base.get_position() - Eigen::Vector3d(0, 0, -0.06));
            arm_ds.set_attractor(ball_in_arm_base);
          }
        }
          break;
        case STATE::PICK_BALL: {
          for (std::size_t i = 0; i < 3000; ++i) {
            break;
          }
          if (robot.pick_ball()) {
            state = STATE::APPROACH_2;
            base_max_vel = 0.2;
            base_target.set_position(0, 0, 0);
            base_ds.set_attractor(base_target);
//            arm_max_lin = 1;
//            arm_max_ang = 1.5;
//            arm_ds.set_gain({200.0, 200.0, 200.0, 50.0, 50.0, 50.0});
            arm_target.set_position(0.5, -0.4, 1.0 + base_to_arm_base.get_position().z());
            arm_target.set_orientation(Eigen::Quaterniond(0.707, 0.707, 0, 0));
            arm_ds.set_attractor(arm_target);
          }
        }
          break;
        case STATE::APPROACH_2: {
          if (robot.get_eef_pose().dist(arm_ds.get_attractor(), CartesianStateVariable::POSE) < 0.02) {
            robot.lock_joints();
            state = APPROACH_3;
            base_max_vel = 1.5;
            base_target.set_position(2, 0, 0);
            base_ds.set_attractor(base_target);
          }
        }
          break;
        case STATE::APPROACH_3: {
          arm_ds.set_attractor(robot.get_eef_pose());
          if (abs(robot.get_base_pose().get_position().x() - 1.3) < 0.05) {
            state = STATE::RELEASE;
          }
        }
          break;
        case STATE::RELEASE: {
          if (base_max_vel < 0.01) {
            base_target.set_position(0, 0 ,0);
            base_ds.set_attractor(base_target);
            base_max_vel = 0.4;
            state = STATE::GO_BACK;
          }
          else if (robot.release()) {
            base_max_vel = 0;
          }
        }
          break;
        case STATE::GO_BACK:
          break;
      }
      CartesianTwist twist = base_ds.evaluate(robot.get_base_pose());
      twist.clamp(base_max_vel, 0);
      robot.publish_base_twist(twist);

      auto ee_pose = robot.get_eef_pose();
      twist = arm_ds.evaluate(ee_pose);
      twist.clamp(arm_max_lin, arm_max_ang);
//      robot.publish_arm_command(robot.inverse_velocity(twist));
      robot.publish_arm_command(ctrl.compute_command(twist, ee_pose, robot.get_jacobian()));
      rate.sleep();
    }
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mobile_manipulator_control");
  ros::NodeHandle node_handle;

  std::string robot_arm_description;
  if (!node_handle.getParam("/robot/robot_arm_description", robot_arm_description)) {
    ROS_ERROR("Could load parameter '/robot/robot_arm_description' from parameter server.");
    return -1;
  }

  std::string urdf_path = "/tmp/robot.urdf";
  Model::create_urdf_from_string(robot_arm_description, urdf_path);

  RobotInterface robot(&node_handle, urdf_path);

  int freq = 500;
  control_loop(robot, freq);
}