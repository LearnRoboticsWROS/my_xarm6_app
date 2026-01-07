#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import quaternion_from_euler

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration


class MoveToPoseCNCCircleNode(Node):
    def __init__(self):
        super().__init__('move_to_pose_cnc_circle_node')

        # -------------------------------
        # CNC PARAMETERS YOU CAN TUNE
        # -------------------------------
        self.base_frame = 'link_base'
        self.group_name = 'xarm6'
        self.ik_link_name = 'link_tcp'
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Circle geometry (meters) in link_base
        self.center_xyz = (0.0, 0.3, 0.15)   # center of the circle in base frame
        self.radius = 0.07                  # circle radius (m)

        # CNC smoothness
        self.num_points = 60                # more points = smoother circle, more IK calls
        self.circle_time = 8.0              # total time for the full circle (sec)

        # Pre-CNC safety move
        self.pre_cnc_z_offset = 0.12        # approach height above drawing plane (m)
        self.pre_cnc_time = 3.0             # time to reach pre-CNC pose (sec)

        # Optional dwell at start/end (like G-code dwell)
        self.start_dwell = 0.0              # seconds
        self.end_dwell = 0.0                # seconds

        # IMPORTANT: collision check sometimes makes IK fail even if pose is reachable
        self.avoid_collisions = True        # try False if IK fails

        # Constant tool orientation (keep fixed to behave like CNC)
        roll, pitch, yaw = (-3.14, 0.0, 0.0)
        self.qx, self.qy, self.qz, self.qw = quaternion_from_euler(roll, pitch, yaw)

        # -------------------------------
        # ROS clients
        # -------------------------------
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.get_logger().info('Waiting for IK service: compute_ik ...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('IK service not available')
        self.get_logger().info('IK service available.')

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm6_traj_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for action server: /xarm6_traj_controller/follow_joint_trajectory ...')
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Trajectory action server not available')
        self.get_logger().info('Trajectory action server available.')

    # -----------------------------
    def wait_future(self, future, timeout_sec=5.0):
        """Process ROS events until future is done (no deadlock)."""
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (time.time() - start) > timeout_sec:
                return False
        return future.done()

    # -----------------------------
    def compute_ik_joints(self, xyz, timeout_sec=3.0):
        x, y, z = xyz

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = self.qx
        pose.pose.orientation.y = self.qy
        pose.pose.orientation.z = self.qz
        pose.pose.orientation.w = self.qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = self.avoid_collisions

        fut = self.ik_client.call_async(req)

        if not self.wait_future(fut, timeout_sec=timeout_sec):
            self.get_logger().error('IK service call timeout.')
            return None

        res = fut.result()
        if res is None or res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f'IK failed (error_code={getattr(res.error_code, "val", None)}).')
            return None

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))

        missing = [j for j in self.arm_joints if j not in name_to_pos]
        if missing:
            self.get_logger().error(f'IK returned missing joints: {missing}')
            return None

        return [float(name_to_pos[j]) for j in self.arm_joints]

    # -----------------------------
    def make_point(self, positions, time_from_start_sec):
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)

        sec = int(time_from_start_sec)
        nanosec = int((time_from_start_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        return pt

    # -----------------------------
    def send_trajectory_goal(self, points, timeout_sec=30.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        goal.trajectory.points = points

        send_future = self.traj_client.send_goal_async(goal)
        if not self.wait_future(send_future, timeout_sec=3.0):
            self.get_logger().error('Send goal timeout.')
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, timeout_sec=timeout_sec):
            self.get_logger().error('Trajectory result timeout.')
            return False

        result = result_future.result().result
        if result.error_code != 0:
            self.get_logger().error(f'Trajectory execution error_code: {result.error_code}')
            return False

        return True

    # -----------------------------
    def run_program(self):
        cx, cy, cz = self.center_xyz

        # Start point on circle (angle = 0): (cx + r, cy, cz)
        start_xyz = (cx + self.radius, cy, cz)

        # Pre-CNC: same XY as start point, higher Z
        pre_cnc_xyz = (start_xyz[0], start_xyz[1], start_xyz[2] + self.pre_cnc_z_offset)

        self.get_logger().info(
            f'1) Pre-CNC target xyz={pre_cnc_xyz}, rpy=(-3.14,0,0), '
            f'avoid_collisions={self.avoid_collisions}'
        )

        pre_joints = self.compute_ik_joints(pre_cnc_xyz, timeout_sec=5.0)
        if pre_joints is None:
            self.get_logger().error(
                'Pre-CNC IK failed. Try avoid_collisions=False or change pre_cnc_z_offset/center_xyz/radius.'
            )
            return False

        # Go to pre-CNC (safe height)
        ok = self.send_trajectory_goal([self.make_point(pre_joints, self.pre_cnc_time)], timeout_sec=10.0)
        if not ok:
            self.get_logger().error('Failed to reach pre-CNC pose.')
            return False

        # Build CNC points for the circle
        self.get_logger().info('2) Building CNC trajectory for circle...')
        traj_points = []
        t = 0.0

        # Descend to drawing plane at start point
        start_joints = self.compute_ik_joints(start_xyz, timeout_sec=5.0)
        if start_joints is None:
            self.get_logger().error('Start point IK failed.')
            return False

        t += 1.0  # descent time (tune)
        traj_points.append(self.make_point(start_joints, t))

        if self.start_dwell > 0.0:
            t += self.start_dwell
            traj_points.append(self.make_point(start_joints, t))

        # Circle interpolation (0 -> 2π)
        # We add points around the circle and optionally close the loop.
        dt = self.circle_time / float(self.num_points)

        for i in range(1, self.num_points + 1):
            theta = (2.0 * math.pi) * (i / float(self.num_points))
            x = cx + self.radius * math.cos(theta)
            y = cy + self.radius * math.sin(theta)
            z = cz

            joints = self.compute_ik_joints((x, y, z), timeout_sec=5.0)
            if joints is None:
                self.get_logger().error(f'IK failed at i={i}/{self.num_points}, theta={theta:.3f}, xyz=({x:.3f},{y:.3f},{z:.3f})')
                return False

            t += dt
            traj_points.append(self.make_point(joints, t))

        if self.end_dwell > 0.0:
            t += self.end_dwell
            traj_points.append(self.make_point(traj_points[-1].positions, t))

        # Lift back up to pre-CNC height
        end_pre = self.compute_ik_joints(pre_cnc_xyz, timeout_sec=5.0)
        if end_pre is None:
            self.get_logger().error('End pre-CNC IK failed.')
            return False

        t += 1.5  # lift time
        traj_points.append(self.make_point(end_pre, t))

        self.get_logger().info('3) Sending ONE CNC-style trajectory goal...')
        ok = self.send_trajectory_goal(traj_points, timeout_sec=60.0)
        if ok:
            self.get_logger().info('✅ CNC circle completed.')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseCNCCircleNode()
    ok = node.run_program()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
