import rclpy
import state_representation as sr
import std_msgs.msg
from dynamical_systems import create_cartesian_ds, DYNAMICAL_SYSTEM
from controllers import create_joint_controller, CONTROLLER_TYPE
from ros2_examples.robot_interface_node import RobotInterfaceNode


class JointSpaceImpedanceControl(RobotInterfaceNode):
    def __init__(self, node_name, dt):
        super().__init__(node_name, "joint_states", "effort_controller/command")
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.init_robot_model(robot_name)
        target = sr.CartesianPose(self.robot.get_frames()[-1], self.robot.get_frames()[0])
        target.set_position([0.6, -0.3, 0.5])
        target.set_orientation([0, 1, 0, 0])
        self._ds = create_cartesian_ds(DYNAMICAL_SYSTEM.POINT_ATTRACTOR)
        self._ds.set_parameter_value("attractor", target, sr.StateType.PARAMETER_CARTESIANPOSE)
        self._ds.set_parameter_value("gain", [50, 50, 50, 10, 10, 10], sr.StateType.PARAMETER_DOUBLE_ARRAY)
        self._ctrl = create_joint_controller(CONTROLLER_TYPE.IMPEDANCE, self.robot)
        self._timer = self.create_timer(dt, self.control_loop)

    def control_loop(self):
        if self.state_received and rclpy.ok():
            twist = sr.CartesianTwist(
                self._ds.evaluate(self.robot.forward_kinematics(sr.JointPositions(self.joint_state))))
            twist.clamp(0.25, 0.25)
            joint_velocities = self.robot.compute_jacobian(sr.JointPositions(self.joint_state)) * twist
            command = self._ctrl.compute_command(joint_velocities, self.joint_state)
            self.publish(command)

    def publish(self, command):
        msg = std_msgs.msg.Float64MultiArray()
        msg.data = command.get_torques().tolist()
        self.publisher.publish(msg)

    def stop(self):
        self.publish(sr.JointTorques().Zero(self.robot.get_robot_name(), self.robot.get_number_of_joints()))
        rclpy.shutdown()


def main():
    rclpy.init()
    node = JointSpaceImpedanceControl('joint_space_impedance_control', 0.01)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
