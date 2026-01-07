#!/usr/bin/env python3
import time
import json
import math
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

from linkattacher_msgs.srv import AttachLink, DetachLink
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from my_xarm6_interfaces.srv import ObjectPosition
from my_xarm6_interfaces.srv import PlacePosition


class GeneralExecutorNode(Node):
    def __init__(self):
        super().__init__('general_executor_node')

        # =========================
        # Map color -> gazebo model/link
        # =========================
        self.model_map = {
            "red":   {"model": "red_cube",       "link": "link_0"},
            "green": {"model": "green_cylinder", "link": "link_1"},
        }

        # =========================
        # Service server: pick_place_vision
        # =========================
        self.pick_place_srv = self.create_service(
            PlacePosition,
            'pick_place_vision',
            self.handle_pick_place
        )

        # =========================
        # Vision client
        # =========================
        self.vision_client = self.create_client(ObjectPosition, 'get_object_position')
        self.get_logger().info("⏳ Waiting for 'get_object_position' service...")
        self.vision_client.wait_for_service()
        self.get_logger().info("✅ 'get_object_position' service is ready.")

        # =========================
        # TF2
        # =========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # IK & Trajectory clients
        # =========================
        self.base_frame = 'link_base'
        self.group_name = 'xarm6'
        self.ik_link_name = 'link_tcp'
        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm6_traj_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm_gripper_traj_controller/follow_joint_trajectory'
        )
        self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_srv = self.create_client(DetachLink, '/DETACHLINK')

        # Wait readiness
        for name, client in [
            ("IK service", self.ik_client),
            ("Arm action", self.arm_client),
            ("Gripper action", self.gripper_client),
            ("AttachLink", self.attach_srv),
            ("DetachLink", self.detach_srv),
        ]:
            if hasattr(client, 'wait_for_service'):
                client.wait_for_service()
            else:
                client.wait_for_server()
            self.get_logger().info(f'{name} ready.')

        # =========================
        # Parameter for Z offset (vision->base)
        # =========================
        self.declare_parameter('z_offset', 0.05)
        self.z_offset = float(self.get_parameter('z_offset').value)
        self.get_logger().info(f"📏 Using Z offset = {self.z_offset:.3f} m")

        # =========================
        # Subscriber: /target_pose (direct motion)
        # =========================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("✅ Subscribed to /target_pose for direct TCP motions.")

        # =========================
        # NEW: Subscriber: /cnc_command (JSON string)
        # =========================
        self.cnc_sub = self.create_subscription(
            String,
            'cnc_command',
            self.cnc_callback,
            10
        )
        self.get_logger().info("✅ Subscribed to /cnc_command for CNC-like motions.")

        self.get_logger().info("🤖 General Executor ready (pick&place + move_to_pose + CNC).")

    # =====================================================================
    #  CNC COMMAND
    # =====================================================================
    def cnc_callback(self, msg: String):
        """Receive CNC command JSON and execute in a separate thread."""
        try:
            params = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"❌ CNC command parse error: {e}")
            self.get_logger().error(f"Raw msg.data: {msg.data}")
            return

        threading.Thread(target=self._run_cnc_job, args=(params,), daemon=True).start()

    def _run_cnc_job(self, params: dict):
        """
        CNC-like motion:
        - fixed orientation
        - build cartesian points
        - compute IK for each
        - send ONE FollowJointTrajectory goal
        """
        shape = str(params.get("shape", "")).lower().strip()
        frame = params.get("frame", "link_base")

        if frame != self.base_frame:
            self.get_logger().warn(f"⚠️ CNC frame '{frame}' not supported, using '{self.base_frame}'.")

        center = params.get("center", {"x": 0.0, "y": 0.3, "z": 0.15})
        cx = float(center.get("x", 0.0))
        cy = float(center.get("y", 0.3))
        cz = float(center.get("z", 0.15))

        pre_z_offset = float(params.get("pre_z_offset", 0.12))
        pre_time = float(params.get("pre_time", 3.0))
        avoid_collisions = bool(params.get("avoid_collisions", True))

        # Fixed orientation
        rpy = params.get("orientation_rpy", None)
        if rpy is None:
            roll, pitch, yaw = (-3.14, 0.0, 0.0)
        else:
            roll = float(rpy.get("roll", -3.14))
            pitch = float(rpy.get("pitch", 0.0))
            yaw = float(rpy.get("yaw", 0.0))
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        self.get_logger().info(
            f"🧾 CNC start: shape={shape}, center=({cx:.3f},{cy:.3f},{cz:.3f}), "
            f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f}), pre_z_offset={pre_z_offset:.3f}, "
            f"avoid_collisions={avoid_collisions}"
        )

        # Build cartesian path points
        if shape == "square":
            sq = params.get("square", {})
            side = float(sq.get("side", 0.10))
            steps_per_edge = int(sq.get("steps_per_edge", 10))
            xyz_list = self._build_square_path(cx, cy, cz, side, steps_per_edge)
            total_time = float(params.get("speed", {}).get("total_time", 8.0))

        elif shape == "rectangle":
            rc = params.get("rectangle", {})
            width = float(rc.get("width", 0.12))
            height = float(rc.get("height", 0.08))
            steps_per_edge = int(rc.get("steps_per_edge", 10))
            xyz_list = self._build_rectangle_path(cx, cy, cz, width, height, steps_per_edge)
            total_time = float(params.get("speed", {}).get("total_time", 8.0))

        elif shape == "circle":
            cc = params.get("circle", {})
            radius = float(cc.get("radius", 0.07))
            num_points = int(cc.get("num_points", 60))
            total_time = float(cc.get("total_time", cc.get("circle_time", 8.0)))
            xyz_list = self._build_circle_path(cx, cy, cz, radius, num_points)

        else:
            self.get_logger().error(f"❌ Unknown CNC shape '{shape}'. Use: square | rectangle | circle")
            return

        # Convert to joint trajectory points (one goal)
        points = self._cartesian_to_joint_trajectory(
            xyz_list=xyz_list,
            quat_xyzw=(qx, qy, qz, qw),
            pre_z_offset=pre_z_offset,
            pre_time=pre_time,
            total_time=total_time,
            avoid_collisions=avoid_collisions
        )
        if not points:
            self.get_logger().error("❌ CNC aborted: could not build joint trajectory.")
            return

        ok = self._send_one_trajectory(points, timeout_sec=max(60.0, total_time + pre_time + 10.0))
        if ok:
            self.get_logger().info("✅ CNC motion completed.")
        else:
            self.get_logger().error("❌ CNC motion failed.")

    # -------------------------
    # CNC geometry builders
    # -------------------------
    def _build_square_path(self, cx, cy, cz, side, steps_per_edge):
        half = side / 2.0
        corners = [
            (cx - half, cy - half, cz),
            (cx + half, cy - half, cz),
            (cx + half, cy + half, cz),
            (cx - half, cy + half, cz),
            (cx - half, cy - half, cz),
        ]
        return self._interpolate_polyline(corners, steps_per_edge)

    def _build_rectangle_path(self, cx, cy, cz, width, height, steps_per_edge):
        hx = width / 2.0
        hy = height / 2.0
        corners = [
            (cx - hx, cy - hy, cz),
            (cx + hx, cy - hy, cz),
            (cx + hx, cy + hy, cz),
            (cx - hx, cy + hy, cz),
            (cx - hx, cy - hy, cz),
        ]
        return self._interpolate_polyline(corners, steps_per_edge)

    def _build_circle_path(self, cx, cy, cz, radius, num_points):
        pts = []
        for i in range(num_points + 1):
            theta = (2.0 * math.pi) * (i / float(num_points))
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)
            pts.append((x, y, cz))
        return pts

    def _interpolate_polyline(self, corners, steps_per_edge):
        pts = [corners[0]]
        for i in range(len(corners) - 1):
            p0 = corners[i]
            p1 = corners[i + 1]
            for step in range(1, steps_per_edge + 1):
                a = step / float(steps_per_edge)
                x = p0[0] + a * (p1[0] - p0[0])
                y = p0[1] + a * (p1[1] - p0[1])
                z = p0[2] + a * (p1[2] - p0[2])
                pts.append((x, y, z))
        return pts

    # -------------------------
    # CNC: IK + trajectory goal
    # -------------------------
    def _cartesian_to_joint_trajectory(
        self,
        xyz_list,
        quat_xyzw,
        pre_z_offset,
        pre_time,
        total_time,
        avoid_collisions
    ):
        if not xyz_list:
            return None

        qx, qy, qz, qw = quat_xyzw
        x0, y0, z0 = xyz_list[0]
        pre_xyz = (x0, y0, z0 + pre_z_offset)

        pre_joints = self._compute_ik_joints(pre_xyz, (qx, qy, qz, qw), avoid_collisions, timeout_sec=6.0)
        if pre_joints is None:
            self.get_logger().error("❌ CNC: Pre pose IK failed.")
            return None

        start_joints = self._compute_ik_joints((x0, y0, z0), (qx, qy, qz, qw), avoid_collisions, timeout_sec=6.0)
        if start_joints is None:
            self.get_logger().error("❌ CNC: Start pose IK failed.")
            return None

        points = []
        t = 0.0

        # 1) Go pre
        t += float(pre_time)
        points.append(self._make_point(pre_joints, t))

        # 2) Down to start
        t += 1.0
        points.append(self._make_point(start_joints, t))

        # 3) Drawing
        n = max(1, len(xyz_list) - 1)
        dt = float(total_time) / float(n)

        for i in range(1, len(xyz_list)):
            joints = self._compute_ik_joints(xyz_list[i], (qx, qy, qz, qw), avoid_collisions, timeout_sec=6.0)
            if joints is None:
                self.get_logger().error(f"❌ CNC IK failed at i={i}/{len(xyz_list)-1}, xyz={xyz_list[i]}")
                return None
            t += dt
            points.append(self._make_point(joints, t))

        # 4) Lift to pre again
        end_pre = self._compute_ik_joints(pre_xyz, (qx, qy, qz, qw), avoid_collisions, timeout_sec=6.0)
        if end_pre is None:
            end_pre = pre_joints

        t += 1.5
        points.append(self._make_point(end_pre, t))

        return points

    def _make_point(self, positions, time_from_start_sec):
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        sec = int(time_from_start_sec)
        nanosec = int((time_from_start_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        return pt

    def _wait_future(self, future, timeout_sec=5.0):
        """Spin while waiting (safe for CNC thread)."""
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (time.time() - start) > timeout_sec:
                return False
        return future.done()

    def _compute_ik_joints(self, xyz, quat_xyzw, avoid_collisions=True, timeout_sec=3.0):
        x, y, z = xyz
        qx, qy, qz, qw = quat_xyzw

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = bool(avoid_collisions)

        fut = self.ik_client.call_async(req)
        if not self._wait_future(fut, timeout_sec=timeout_sec):
            self.get_logger().error("❌ IK service timeout.")
            return None

        res = fut.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            return None

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))

        missing = [j for j in self.arm_joints if j not in name_to_pos]
        if missing:
            self.get_logger().error(f"❌ IK returned missing joints: {missing}")
            return None

        return [float(name_to_pos[j]) for j in self.arm_joints]

    def _send_one_trajectory(self, points, timeout_sec=60.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        goal.trajectory.points = points

        send_future = self.arm_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=3.0):
            self.get_logger().error("❌ CNC: send_goal timeout.")
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("❌ CNC: goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=timeout_sec):
            self.get_logger().error("❌ CNC: result timeout.")
            return False

        result = result_future.result().result
        if result.error_code != 0:
            self.get_logger().error(f"❌ CNC: execution error_code={result.error_code}")
            return False

        return True

    # =====================================================================
    #  PICK & PLACE SERVICE 
    # =====================================================================
    def handle_pick_place(self, request, response):
        color = request.object_name.lower().strip()
        place_target = [request.x, request.y, request.z]

        if (request.pick_roll == 0.0 and
            request.pick_pitch == 0.0 and
            request.pick_yaw == 0.0):
            pick_rpy = (-3.141, 0.0, 0.0)
        else:
            pick_rpy = (request.pick_roll, request.pick_pitch, request.pick_yaw)

        if (request.place_roll == 0.0 and
            request.place_pitch == 0.0 and
            request.place_yaw == 0.0):
            place_rpy = pick_rpy
        else:
            place_rpy = (request.place_roll, request.place_pitch, request.place_yaw)

        self.get_logger().info(
            f"🎯 Received pick & place request from LLM: color={color}, place={place_target}, "
            f"pick_rpy={pick_rpy}, place_rpy={place_rpy}"
        )

        threading.Thread(
            target=self._run_pick_place_sequence,
            args=(color, place_target, pick_rpy, place_rpy),
            daemon=True
        ).start()

        response.success = True
        return response

    def _run_pick_place_sequence(self, color: str, place_target, pick_rpy, place_rpy):
        self.get_logger().info(f"▶️ Starting pick&place thread for color={color}...")

        grasp_pose = self.get_object_position_base(color)
        if not grasp_pose:
            self.get_logger().error(f"❌ Could not get position for object '{color}'. Aborting.")
            return

        if color in self.model_map:
            model = self.model_map[color]["model"]
            link = self.model_map[color]["link"]
        else:
            self.get_logger().warn(f"⚠️ Unknown color '{color}', using default model/link.")
            model, link = f"{color}_object", "link_0"

        self.get_logger().info(f"🧩 Using Gazebo object: {model}::{link}")

        z_approach_offset = 0.05
        pre_pose = [grasp_pose[0], grasp_pose[1], grasp_pose[2] + z_approach_offset]
        post_pose = [grasp_pose[0], grasp_pose[1], grasp_pose[2] + z_approach_offset]

        roll_p, pitch_p, yaw_p = pick_rpy or (0.0, 0.0, 0.0)
        roll_pl, pitch_pl, yaw_pl = place_rpy or (roll_p, pitch_p, yaw_p)

        self.get_logger().info(
            f"🎯 Final pick pose: {grasp_pose} | RPY={pick_rpy}, "
            f"place={place_target} | place_rpy={place_rpy}"
        )

        try:
            self.pick_and_place(
                model=model,
                link=link,
                grasp=grasp_pose,
                pre=pre_pose,
                post=post_pose,
                place=place_target,
                rpy_pick=(roll_p, pitch_p, yaw_p),
                rpy_place=(roll_pl, pitch_pl, yaw_pl)
            )
            self.get_logger().info(f"✅ Pick & place of {color} completed.")
        except Exception as e:
            self.get_logger().error(f"❌ Exception in pick&place sequence: {e}")

    def get_object_position_base(self, object_name):
        req = ObjectPosition.Request()
        req.object_name = object_name
        future = self.vision_client.call_async(req)

        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        res = future.result()
        if not res or not res.success:
            self.get_logger().error(f"Failed to get position of {object_name}")
            return None

        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_optical_link'
        point_cam.point.x = res.x
        point_cam.point.y = res.y
        point_cam.point.z = res.z

        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',
                'camera_optical_link',
                rclpy.time.Time()
            )
            point_base = do_transform_point(point_cam, transform)
            xyz = [
                point_base.point.x,
                point_base.point.y,
                point_base.point.z + self.z_offset
            ]
            self.get_logger().info(f"📍 Object '{object_name}' in base frame (with z_offset): {xyz}")
            return xyz
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def compute_ik(self, *args):
        if len(args) == 1 and isinstance(args[0], Pose):
            pose_input = args[0]
        elif len(args) == 6:
            x, y, z, roll, pitch, yaw = args
            pose_input = Pose()
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            pose_input.position.x = x
            pose_input.position.y = y
            pose_input.position.z = z
            pose_input.orientation.x = qx
            pose_input.orientation.y = qy
            pose_input.orientation.z = qz
            pose_input.orientation.w = qw
        else:
            self.get_logger().error("Invalid arguments for compute_ik()")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'link_base'
        pose_stamped.pose = pose_input

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        res = future.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error("IK failed.")
            return None

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        return [name_to_pos[j] for j in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']]

    def move_arm(self, positions, duration=3.0):
        if not positions:
            self.get_logger().error("Invalid joint positions, skipping arm motion.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)

        future = self.arm_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        handle = future.result()
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

    def move_gripper(self, position, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['drive_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)

        future = self.gripper_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        handle = future.result()
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

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

    def pick_and_place(self, model, link, grasp, pre, post, place, rpy_pick, rpy_place):
        self.get_logger().info(f"🦾 Executing pick_and_place for {model}::{link}")

        # Pick quaternion
        qx_p, qy_p, qz_p, qw_p = quaternion_from_euler(*rpy_pick)

        pose_pick = Pose()
        pose_pick.position.x = grasp[0]
        pose_pick.position.y = grasp[1]
        pose_pick.position.z = grasp[2]
        pose_pick.orientation.x = qx_p
        pose_pick.orientation.y = qy_p
        pose_pick.orientation.z = qz_p
        pose_pick.orientation.w = qw_p

        pose_pre = Pose()
        pose_pre.position.x = pre[0]
        pose_pre.position.y = pre[1]
        pose_pre.position.z = pre[2]
        pose_pre.orientation = pose_pick.orientation

        pose_post = Pose()
        pose_post.position.x = post[0]
        pose_post.position.y = post[1]
        pose_post.position.z = post[2]
        pose_post.orientation = pose_pick.orientation

        # Place quaternion
        qx_pl, qy_pl, qz_pl, qw_pl = quaternion_from_euler(*rpy_place)
        pose_place = Pose()
        pose_place.position.x = place[0]
        pose_place.position.y = place[1]
        pose_place.position.z = place[2]
        pose_place.orientation.x = qx_pl
        pose_place.orientation.y = qy_pl
        pose_place.orientation.z = qz_pl
        pose_place.orientation.w = qw_pl

        self.get_logger().info(f"📦 pick_rpy={rpy_pick}  place_rpy={rpy_place}")

        # Sequence
        self.get_logger().info("➡️ Pre-pick...")
        joint_pre = self.compute_ik(pose_pre)
        if joint_pre: self.move_arm(joint_pre)

        self.get_logger().info("⬇️ Pick...")
        joint_pick = self.compute_ik(pose_pick)
        if joint_pick: self.move_arm(joint_pick)

        self.get_logger().info("✋ Close + attach...")
        self.move_gripper(0.17)
        self.attach_object(model, link)

        self.get_logger().info("⬆️ Post-pick...")
        joint_post = self.compute_ik(pose_post)
        if joint_post: self.move_arm(joint_post)

        self.get_logger().info("➡️ Place...")
        joint_place = self.compute_ik(pose_place)
        if joint_place: self.move_arm(joint_place)

        self.get_logger().info("🖐 Open + detach...")
        self.move_gripper(0.0)
        self.detach_object(model, link)

        self.get_logger().info("🔁 Return post...")
        if joint_post: self.move_arm(joint_post)

        self.get_logger().info("✅ pick_and_place completed.")

    # =====================================================================
    #  Direct motion from /target_pose 
    # =====================================================================
    def pose_callback(self, pose_msg: PoseStamped):
        self.get_logger().info("📩 [LLM] Received direct pose target.")
        self.send_ik_request(pose_msg)

    def send_ik_request(self, pose_msg: PoseStamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose_msg
        req.ik_request.avoid_collisions = True

        self.get_logger().info("🧮 [LLM] Requesting IK for direct pose...")
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ IK service call failed: {e}")
            return

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f"❌ IK failed, code: {res.error_code.val}")
            return

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        ordered_positions = [name_to_pos[j] for j in self.arm_joints if j in name_to_pos]
        self.get_logger().info(f"✅ [LLM] IK solution found.")
        self.send_trajectory(self.arm_joints, ordered_positions)

    def send_trajectory(self, joint_names, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(point)

        self.get_logger().info("🚀 [LLM] Sending trajectory...")
        send_future = self.arm_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return
        self.get_logger().info("Trajectory accepted. Waiting result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("✅ Trajectory execution finished.")
        self.get_logger().info(f"Result error_code: {result.error_code}")


def main(args=None):
    rclpy.init(args=args)
    node = GeneralExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
