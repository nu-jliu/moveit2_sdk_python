import rclpy
import rclpy.node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    JointConstraint,
)
from geometry_msgs.msg import Pose, Point, Quaternion


from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup

from franka_msgs.action import Grasp, Homing

import threading
import copy

import google_speech


class Moveit2Python:
    def __init__(self, base_frame, ee_frame, group_name):
        self.node = rclpy.create_node("moveit2_cli_node")
        self.base_frame_id = base_frame
        self.ee_frame_id = ee_frame
        self.group_name = group_name
        self.callback_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.robot_state = RobotState()

        self.sub_joint_states = self.node.create_subscription(
            JointState,
            "joint_states",
            self.sub_joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.cli_get_cartesian_path = self.node.create_client(
            GetCartesianPath,
            "compute_cartesian_path",
            callback_group=self.callback_group,
        )

        self.cli_action_execute_trajectory = ActionClient(
            self.node,
            ExecuteTrajectory,
            "execute_trajectory",
            callback_group=self.callback_group,
        )

        self.cli_action_move_action = ActionClient(
            self.node,
            MoveGroup,
            "move_action",
            callback_group=self.callback_group,
        )

        self.cli_action_grasp = ActionClient(
            self.node,
            Grasp,
            "panda_gripper/grasp",
            callback_group=self.callback_group,
        )

        while not self.cli_get_cartesian_path.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                f"Service {self.cli_get_cartesian_path.srv_name} not available, waiting again ..."
            )

        # while not self.cli_action_grasp.wait_for_server(timeout_sec=2.0):
        #     self.node.get_logger().info("waiting")

        while self.node.count_publishers("joint_states") < 1:
            self.node.get_logger().warn(
                "Not enoutgh joint state message received, waiting again ..."
            )

        self.node.get_logger().info("API initialized")

        self.thread_node = threading.Thread(target=self.spin_node, daemon=True)
        self.thread_node.start()

    def __del__(self):
        self.node.get_logger().info("Shutting down api node")
        self.thread_node.join()
        self.node.destroy_node()
        rclpy.try_shutdown()

    def spin_node(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        rclpy.spin(node=self.node, executor=self.executor)

    def sub_joint_state_callback(self, msg: JointState):
        self.robot_state.joint_state = msg

    async def move_group(self, names, positions):
        self.node.get_logger().info("Sending request to move group")
        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        ws_params = WorkspaceParameters()

        ws_params.header.stamp = self.node.get_clock().now().to_msg()
        ws_params.header.frame_id = self.base_frame_id

        ws_params.min_corner.x = -1.0
        ws_params.min_corner.y = -1.0
        ws_params.min_corner.z = -1.0

        ws_params.max_corner.x = 1.0
        ws_params.max_corner.y = 1.0
        ws_params.max_corner.z = 1.0

        request.workspace_parameters = ws_params
        request.start_state = self.robot_state

        goal_constraint = Constraints()
        joint_constraints = []

        for name, position in zip(names, positions):
            consraint = JointConstraint()
            consraint.joint_name = name
            consraint.position = position
            consraint.tolerance_above = 0.01
            consraint.tolerance_below = 0.01
            consraint.weight = 1.0

            joint_constraints.append(consraint)

        goal_constraint.name = ""
        goal_constraint.joint_constraints = joint_constraints
        request.goal_constraints = [goal_constraint]

        request.pipeline_id = "move_group"
        request.group_name = self.group_name
        request.num_planning_attempts = 40
        request.allowed_planning_time = 20.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = True

        future = self.cli_action_move_action.send_goal_async(goal)
        await future

        future = future.result().get_result_async()
        await future

        result = future.result()
        self.node.get_logger().info("Move group finished")
        return result

    async def get_cartesian_path(self, waypoints: list[Pose]):
        self.node.get_logger().info("Sending request to get cartesian path")
        request = GetCartesianPath.Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.header.frame_id = self.base_frame_id
        request.start_state = self.robot_state
        request.group_name = self.group_name
        request.link_name = self.ee_frame_id
        request.waypoints = waypoints

        request.max_step = 0.01
        request.avoid_collisions = True
        request.max_cartesian_speed = 0.08
        request.max_velocity_scaling_factor = 0.08
        request.max_acceleration_scaling_factor = 0.03
        request.cartesian_speed_limited_link = self.ee_frame_id

        future = self.cli_get_cartesian_path.call_async(request)
        # rclpy.spin_until_future_complete(
        #     node=self.node,
        #     future=future,
        #     executor=self.executor,
        #     timeout_sec=10.0,
        # )
        await future

        result = future.result()
        self.node.get_logger().info("Get cartesian path finished")
        return result.solution

    async def execute_trajectory(self, trajectory: RobotTrajectory):
        self.node.get_logger().info("Sending request to execute trajectory")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.cli_action_execute_trajectory.send_goal_async(goal)
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

        result = future.result()
        self.node.get_logger().info("Executing trajectory finished")
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

        result = future.result()
        self.node.get_logger().info("Grasp finished")
        return result

    async def pour(self, position: Point):
        self.node.get_logger().info("Starting pouring water")
        speech = google_speech.Speech("Pouring", "en")
        speech.play()

        planned_trajectory = await self.get_cartesian_path(
            [
                Pose(
                    position=position,
                    orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
                )
            ]
        )
        await self.execute_trajectory(planned_trajectory)

        ori_pour = Quaternion()
        ori_pour.x = -0.923879532511287
        ori_pour.y = 0.0
        ori_pour.z = 0.382683432365090
        ori_pour.w = 0.0

        planned_trajectory = await self.get_cartesian_path(
            [Pose(position=position, orientation=ori_pour)]
        )
        result = await self.execute_trajectory(planned_trajectory)
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

        planned_traj = await self.get_cartesian_path([pose1, pose2])
        result = await self.execute_trajectory(planned_traj)

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

        result = await self.move_group(names, positions)
        return result
