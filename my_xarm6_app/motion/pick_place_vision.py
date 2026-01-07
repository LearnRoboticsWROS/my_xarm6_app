#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener
import time

from my_xarm6_interfaces.srv import ObjectPosition

class PickPlaceVisionNode(Node):
    def __init__(self):
        super().__init__('pick_place_vision_node')

        # --- Service client for vision ---
        self.vision_client = self.create_client(ObjectPosition, 'get_object_position')
        self.vision_client.wait_for_service()

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- IK & Trajectory clients ---
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/xarm6_traj_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/xarm_gripper_traj_controller/follow_joint_trajectory')
        self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_srv = self.create_client(DetachLink, '/DETACHLINK')

        for name, client in [
            ("IK service", self.ik_client),
            ("Arm action", self.arm_client),
            ("Gripper action", self.gripper_client),
            ("AttachLink", self.attach_srv),
            ("DetachLink", self.detach_srv),
        ]:
            client.wait_for_service() if hasattr(client, 'wait_for_service') else client.wait_for_server()
            self.get_logger().info(f'{name} ready.')

        self.execute_sequence()

    def get_object_position_base(self, object_name):
        """request position in 3D and transform with the respect of link_base"""
        req = ObjectPosition.Request()
        req.object_name = object_name
        future = self.vision_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res.success:
            self.get_logger().error(f'Failed to get position of {object_name}')
            return None

        # camera_optical_link → link_base
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_optical_link'
        point_cam.point.x = res.x
        point_cam.point.y = res.y
        point_cam.point.z = res.z

        try:
            transform = self.tf_buffer.lookup_transform('link_base', 'camera_optical_link', rclpy.time.Time())
            from tf2_geometry_msgs import do_transform_point
            point_base = do_transform_point(point_cam, transform)
            return [point_base.point.x, point_base.point.y, point_base.point.z]
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None
        
    # ---------------------------------------------------------------------
    def compute_ik(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = qx, qy, qz, qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            return None

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        return [name_to_pos[j] for j in ['joint1','joint2','joint3','joint4','joint5','joint6']]
    
    # ---------------------------------------------------------------------
    def move_arm(self, positions, duration=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        rclpy.spin_until_future_complete(self, handle.get_result_async())

    def move_gripper(self, position, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['drive_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        rclpy.spin_until_future_complete(self, handle.get_result_async())

    # ---------------------------------------------------------------------

    def attach_object(self, model, link):
        req = AttachLink.Request()
        req.model1_name, req.link1_name = 'UF_ROBOT', 'link6'
        req.model2_name, req.link2_name = model, link
        self.attach_srv.call_async(req)

    def detach_object(self, model, link):
        req = DetachLink.Request()
        req.model1_name, req.link1_name = 'UF_ROBOT', 'link6'
        req.model2_name, req.link2_name = model, link
        self.detach_srv.call_async(req)

    # ---------------------------------------------------------------------


    def execute_sequence(self):
        roll, pitch, yaw = (-3.141, 0.0, 0.0)

        # === Red cube ===
        grasp_cube = self.get_object_position_base('red')
        grasp_cube_offset = [grasp_cube[0], grasp_cube[1], grasp_cube[2]+ 0.05]
        if grasp_cube:
            pre_cube = [grasp_cube[0], grasp_cube[1], grasp_cube[2] + 0.1]
            post_cube = pre_cube
            place_cube = [0.3, 0.3, 0.1]
            self.pick_and_place('red_cube', 'link_0', grasp_cube_offset, pre_cube, post_cube, place_cube, (roll,pitch,yaw))

        # === Green cylinder ===
        grasp_cyl = self.get_object_position_base('green')
        if grasp_cyl:
            pre_cyl = [grasp_cyl[0], grasp_cyl[1], grasp_cyl[2] + 0.1]
            post_cyl = pre_cyl
            place_cyl = [-0.3, 0.3, 0.1]
            self.pick_and_place('green_cylinder', 'link_1', grasp_cyl, pre_cyl, post_cyl, place_cyl, (roll,pitch,yaw))

    # ---------------------------------------------------------------------
    def pick_and_place(self, model, link, grasp, pre, post, place, rpy):
        roll, pitch, yaw = rpy
        for pose in [pre, grasp]:
            pos = self.compute_ik(*pose, roll, pitch, yaw)
            self.move_arm(pos)
        self.move_gripper(0.17)
        self.attach_object(model, link)
        pos = self.compute_ik(*post, roll, pitch, yaw)
        self.move_arm(pos)
        pos = self.compute_ik(*place, roll, pitch, yaw)
        self.move_arm(pos)
        self.move_gripper(0.0)
        self.detach_object(model, link)
        self.move_arm(post)





def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceVisionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()