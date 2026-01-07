#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import euler_from_quaternion
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration


class MoveToPositionFromLLM(Node):
    def __init__(self):
        super().__init__('move_to_position_from_llm')

        # 1️⃣ Subscribe to /target_pose published by the LLM node
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.pose_callback,
            10
        )

        # 2️⃣ Service client for IK
        self.ik_service = 'compute_ik'
        self.ik_client = self.create_client(GetPositionIK, self.ik_service)
        self.get_logger().info(f'Waiting for IK service: {self.ik_service}...')
        self.ik_client.wait_for_service()

        # 3️⃣ Action client for trajectory
        self.traj_action = '/xarm6_traj_controller/follow_joint_trajectory'
        self.traj_client = ActionClient(self, FollowJointTrajectory, self.traj_action)
        self.get_logger().info(f'Waiting for trajectory action: {self.traj_action}...')
        self.traj_client.wait_for_server()

        self.get_logger().info('✅ Ready to receive poses from LLM.')

    # ---- callback ----
    def pose_callback(self, pose_msg: PoseStamped):
        self.get_logger().info(f'📩 Received target pose from LLM.')
        self.send_ik_request(pose_msg)

    def send_ik_request(self, pose: PoseStamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True

        self.get_logger().info('🧮 Requesting IK solution...')
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f'IK failed, code: {res.error_code.val}')
            return

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        ordered_positions = [name_to_pos[j] for j in arm_joints if j in name_to_pos]

        self.get_logger().info(f'✅ IK solution: {ordered_positions}')
        self.send_trajectory(arm_joints, ordered_positions)

    def send_trajectory(self, joint_names, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(point)

        self.get_logger().info('🚀 Sending trajectory to controller...')
        send_future = self.traj_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            return

        self.get_logger().info('Trajectory accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('✅ Trajectory execution finished.')
        self.get_logger().info(f'Result error_code: {result.error_code}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPositionFromLLM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()