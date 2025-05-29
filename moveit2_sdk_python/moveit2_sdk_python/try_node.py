import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit2_sdk_python.moveit2_sdk_python import Moveit2Python
from moveit2_sdk_python.franka_mover import FrankaMover

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes

from std_srvs.srv import Empty, Empty_Request, Empty_Response


class TryNode(Node):

    def __init__(self):
        """Initialize the TryNode class.

        This constructor initializes the ROS 2 node, creates instances of
        Moveit2Python and FrankaMover, sets up a reentrant callback group,
        subscribes to joint states, and creates a service to test Cartesian paths.
        """
        super().__init__("try_node")
        self.api = Moveit2Python(
            base_frame="panda_link0",
            ee_frame="panda_hand_tcp",
            group_name="panda_manipulator",
        )
        self.franka_mover = FrankaMover()

        self.callback_group = ReentrantCallbackGroup()

        self.joint_states: JointState = None

        self.sub_joint_states = self.create_subscription(
            JointState,
            "joint_states",
            self.sub_joint_states_callback,
            10,
            callback_group=self.callback_group,
        )

        self.srv_test_cartesian = self.create_service(
            Empty,
            "test_cartesian",
            self.srv_test_cartesian_callback,
            callback_group=self.callback_group,
        )

    def sub_joint_states_callback(self, msg: JointState):
        """Callback function for the joint state subscriber.

        Updates the internal joint_states with the received joint state message.

        :param msg: The received JointState message.
        :type msg: JointState
        """
        self.joint_states = msg

    async def srv_test_cartesian_callback(
        self,
        request: Empty_Request,
        response: Empty_Response,
    ):
        """Service callback to test a Cartesian path execution.

        This function defines a series of waypoints, computes a Cartesian path,
        executes the trajectory, performs a pouring motion, moves to another position,
        and finally moves the robot to its home position.

        :param request: The Empty request message.
        :type request: Empty_Request
        :param response: The Empty response message.
        :type response: Empty_Response
        :return: The Empty response message.
        :rtype: Empty_Response
        """

        if self.joint_states is None:
            return response

        waypoint = [
            Pose(
                position=Point(x=0.4, y=0.2, z=0.4),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            ),
            Pose(
                position=Point(x=0.4, y=0.0, z=0.4),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            ),
            Pose(
                position=Point(x=0.4, y=-0.2, z=0.4),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            ),
        ]

        trajectory = await self.api.get_cartesian_path(waypoint)
        result = await self.api.execute_trajectory(trajectory)

        result = await self.franka_mover.pour(
            waypoint[-1].position.x,
            waypoint[-1].position.y,
            waypoint[-1].position.z,
        )

        result = await self.franka_mover.move(x=0.5, y=0.0, z=0.3)

        # result = await self.api.grasp()

        # result = await self.api.grasp()

        result = await self.franka_mover.home()

        self.get_logger().info(f"{result.error_code}")
        return response


def main(args=None):
    """Main function to initialize and run the TryNode.

    Initializes RCLPY, creates an instance of TryNode, and spins the node
    to process callbacks. Handles KeyboardInterrupt for clean shutdown.

    :param args: Command line arguments.
    :type args: list, optional
    """
    rclpy.init(args=args)
    node = TryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
