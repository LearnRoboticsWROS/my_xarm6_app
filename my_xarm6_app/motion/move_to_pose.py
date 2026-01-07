#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import quaternion_from_euler

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_node')

        # Client for service
        self.ik_service_name = 'compute_ik'
        self.ik_client = self.create_client(GetPositionIK, self.ik_service_name)
        self.get_logger().info(f'Waiting for IK service: {self.ik_service_name} ...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available, aborting.')
            raise RuntimeError('IK service not available')
        self.get_logger().info('IK service available.')

        # Client for action controller
        self.traj_action_name = '/xarm6_traj_controller/follow_joint_trajectory'
        self.traj_client = ActionClient(self, FollowJointTrajectory, self.traj_action_name)
        self.get_logger().info(f'Connecting to action server: {self.traj_action_name} ...')
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory action server not available, aborting.')
            raise RuntimeError('Trajectory action server not available')
        self.get_logger().info('Trajectory action server available.')

        # 3️ Timer for execution one time
        self.timer = self.create_timer(1.0, self.run_once)
        self._done = False

    def run_once(self):
        if self._done:
            return
        self._done = True

        self.get_logger().info('Sending IK request...')

        # ✅ Frame of reference
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.header.stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = quaternion_from_euler(-2.292, 0.0, -1.57)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.5
        pose.pose.position.z = 0.5

        # Request IK
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)
    

    def ik_response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f'IK failed, error code: {res.error_code.val}')
            return

        js = res.solution.joint_state
        if not js.name:
            self.get_logger().error('IK result empty joint state')
            return

        # order and filter the joints
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        name_to_pos = dict(zip(js.name, js.position))
        ordered_positions = [name_to_pos[j] for j in arm_joints if j in name_to_pos]

        self.get_logger().info(f'IK SUCCESS → ordered positions: {ordered_positions}')

        # build the goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = arm_joints

        point = JointTrajectoryPoint()
        point.positions = ordered_positions
        point.time_from_start = Duration(sec=3)
        goal_msg.trajectory.points.append(point)

        # Send the goal as action
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available!')
            return

        self.get_logger().info('Sending trajectory goal...')
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected by controller.')
            return

        self.get_logger().info('Trajectory goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Trajectory execution finished.')
        self.get_logger().info(f'Result error_code: {result.error_code}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()