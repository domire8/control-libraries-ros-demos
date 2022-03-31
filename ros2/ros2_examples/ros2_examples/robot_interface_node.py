import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from robot_model import Model, create_urdf_from_string
from sensor_msgs.msg import JointState as JointStateMsg
from state_representation import JointState
from std_msgs.msg import Float64MultiArray


class RobotInterfaceNode(Node):
    def __init__(self, node_name, subscription_topic, publisher_topic):
        super().__init__(node_name)
        self.declare_parameter("robot_name", "robot")
        self.publisher = self.create_publisher(Float64MultiArray, publisher_topic, 10)
        self.robot = None
        self.joint_state = None
        self.state_received = False

        self._subscription_topic = subscription_topic
        self._subscription = None

    def init_robot_model(self, name):
        client = self.create_client(GetParameters, "/" + name + "/robot_state_publisher/get_parameters")
        while not client.wait_for_service(1):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                rclpy.shutdown()
            self.get_logger().info("Service not available, waiting again...")
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        robot_description = future.result().values[0].string_value
        urdf_path = "/tmp/" + name + ".urdf"
        create_urdf_from_string(robot_description, urdf_path)
        self.robot = Model(name, urdf_path)
        self.joint_state = JointState(self.robot.get_robot_name(), self.robot.get_joint_frames())
        self._subscription = self.create_subscription(JointStateMsg, self._subscription_topic,
                                                      self._robot_state_callback, 10)

    def _robot_state_callback(self, msg):
        if not self.state_received:
            self.state_received = True
        self.joint_state.set_positions(msg.position)
        self.joint_state.set_velocities(msg.velocity)
        self.joint_state.set_torques(msg.effort)
