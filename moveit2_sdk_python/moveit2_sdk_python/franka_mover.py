import rclpy
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

import google_speech

from moveit2_sdk_python.moveit2_sdk_python import Moveit2Python

from geometry_msgs.msg import Pose, Point, Quaternion
from franka_msgs.action import Grasp

import threading
import random


class FrankaMover:
    def __init__(self) -> None:
        self.api = Moveit2Python(
            base_frame="panda_link0",
            ee_frame="panda_hand_tcp",
            group_name="panda_manipulator",
        )
        self.node = rclpy.create_node(f"franka_mover_{random.randint(0, 1000)}")
        self.callback_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()

        self.cli_action_grasp = ActionClient(
            self.node,
            Grasp,
            "panda_gripper/grasp",
            callback_group=self.callback_group,
        )

        self.thread_node = threading.Thread(target=self.spin_node, daemon=True)
        self.thread_node.start()

        self.node.get_logger().info("Franka mover initialized")

    def __del__(self):
        self.node.get_logger().info("Shutting down franka mover node")
        self.thread_node.join()
        self.node.destroy_node()
        rclpy.try_shutdown()

    def spin_node(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        rclpy.spin(node=self.node, executor=self.executor)

    async def pour(self, x: float, y: float, z: float):
        self.node.get_logger().info("Starting pouring water")
        speech = google_speech.Speech("Pouring", "en")
        speech.play()

        position = Point()
        position.x = x
        position.y = y
        position.z = z

        planned_trajectory = await self.api.get_cartesian_path(
            [
                Pose(
                    position=position,
                    orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
                )
            ]
        )
        await self.api.execute_trajectory(planned_trajectory)

        ori_pour = Quaternion()
        ori_pour.x = -0.923879532511287
        ori_pour.y = 0.0
        ori_pour.z = 0.382683432365090
        ori_pour.w = 0.0

        planned_trajectory = await self.api.get_cartesian_path(
            [Pose(position=position, orientation=ori_pour)]
        )
        result = await self.api.execute_trajectory(planned_trajectory)
        self.node.get_logger().info("Finished pouring water")
        return result

    async def move(self, x: float, y: float, z: float):
        self.node.get_logger().info(f"Starting moving to {x, y, z}")
        speech = google_speech.Speech(f"Moving to {x, y, z}", "en")
        speech.play()

        pose1 = Pose()
        pose1.position.x = x
        pose1.position.y = y
        pose1.position.z = z + 0.1
        pose1.orientation.x = 1.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 0.0

        pose2 = Pose()
        pose2.position.x = x
        pose2.position.y = y
        pose2.position.z = z
        pose2.orientation.x = 1.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 0.0

        planned_traj = await self.api.get_cartesian_path([pose1, pose2])
        result = await self.api.execute_trajectory(planned_traj)

        self.node.get_logger().info(f"Finished move to position {x, y, z}")
        return result

    async def home(self):
        speech = google_speech.Speech("Homing", "en")
        speech.play()

        names = [f"panda_joint{i+1}" for i in range(7)]
        positions = [
            0.0,
            -0.7853981633974483,
            0.0,
            -2.356194490192345,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
        ]

        result = await self.api.move_group(names, positions)
        return result

    async def grasp(self):
        self.node.get_logger().info("Sending request to grasp")
        speech = google_speech.Speech("Grasping", "en")
        speech.play()

        goal = Grasp.Goal()
        goal.width = 0.04
        goal.force = 50.0
        goal.speed = 0.1
        goal.epsilon.inner = 0.004
        goal.epsilon.outer = 0.004

        future = self.cli_action_grasp.send_goal_async(goal)
        # rclpy.spin_until_future_complete(
        #     node=self.node,
        #     future=future,
        #     timeout_sec=10.0,
        # )
        await future

        future = future.result().get_result_async()
        # rclpy.spin_until_future_complete(
        #     node=self.node,
        #     future=future,
        #     timeout_sec=10.0,
        # )
        await future

        result = future.result().result
        self.node.get_logger().info("Grasp finished")
        return result
