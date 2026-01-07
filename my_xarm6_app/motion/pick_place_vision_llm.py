#!/usr/bin/env python3
import time

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
from tf2_geometry_msgs import do_transform_point

from my_xarm6_interfaces.srv import ObjectPosition
from my_xarm6_interfaces.srv import PlacePosition
from geometry_msgs.msg import Pose

class PickPlaceVisionLLMNode(Node):
    def __init__(self):
        super().__init__('pick_place_vision_llm_node')

        # Color map for Gazebo
        self.model_map = {
            "red":   {"model": "red_cube",       "link": "link_0"},
            "green": {"model": "green_cylinder", "link": "link_1"},
        }

        # PlacePosition.srv:
        # string object_name
        # float64 x
        # float64 y
        # float64 z
        # ---
        # bool success
        self.pick_place_srv = self.create_service(
            PlacePosition,
            'pick_place_vision',
            self.handle_pick_place
        )

        # --- Client for vision ---
        self.vision_client = self.create_client(ObjectPosition, 'get_object_position')
        self.get_logger().info("⏳ Waiting for 'get_object_position' service...")
        self.vision_client.wait_for_service()
        self.get_logger().info("✅ 'get_object_position' service is ready.")

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- IK & Trajectory clients ---
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

        # wait all the services are on
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

        self.get_logger().info("🤖 PickPlaceVision LLM Node ready. Waiting for LLM service calls.")

        # Offset in Z
        self.declare_parameter('z_offset', 0.05)
        self.z_offset = self.get_parameter('z_offset').value
        self.get_logger().info(f"📏 Using Z offset = {self.z_offset:.3f} m")

        # --- Subscriber for /target_pose (for direct LLM motion control) ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("✅ Subscribed to /target_pose for direct TCP motions.")

    # =====================================================================
    #  CALLBACK OF SERVECE CALLED from llm_task_node
    # =====================================================================

    def handle_pick_place(self, request, response):
        color = request.object_name.lower().strip()
        place_target = [request.x, request.y, request.z]

        # ✅ Interpret 0,0,0 as not specified and use as default

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

        import threading
        threading.Thread(
            target=self._run_pick_place_sequence,
            args=(color, place_target, pick_rpy, place_rpy),
            daemon=True
        ).start()

        response.success = True
        return response
    
    # =====================================================================
    #  LOGIC OF PICK & PLACE (in thread separated)
    # =====================================================================

    def _run_pick_place_sequence(self, color: str, place_target, pick_rpy, place_rpy):
        self.get_logger().info(f"▶️ Starting pick&place thread for color={color}...")

        # 1️⃣ get the position of the object link_base
        grasp_pose = self.get_object_position_base(color)
        if not grasp_pose:
            self.get_logger().error(f"❌ Could not get position for object '{color}'. Aborting.")
            return

        # 2️⃣ Color map
        if color in self.model_map:
            model = self.model_map[color]["model"]
            link = self.model_map[color]["link"]
        else:
            self.get_logger().warn(f"⚠️ Unknown color '{color}', using default model/link.")
            model, link = f"{color}_object", "link_0"

        self.get_logger().info(f"🧩 Using Gazebo object: {model}::{link}")

        # 3️⃣ adattive logic
        z_approach_offset = 0.05  
        pre_pose = [grasp_pose[0], grasp_pose[1], grasp_pose[2] + z_approach_offset]
        post_pose = [grasp_pose[0], grasp_pose[1], grasp_pose[2] + z_approach_offset]

        # 4️⃣ RPY
        roll_p, pitch_p, yaw_p = pick_rpy or (0.0, 0.0, 0.0)
        roll_pl, pitch_pl, yaw_pl = place_rpy or (roll_p, pitch_p, yaw_p)

        self.get_logger().info(
            f"🎯 Final pick pose: {grasp_pose} | RPY={pick_rpy}, "
            f"place={place_target} | place_rpy={place_rpy}"
        )

        # 5️⃣ Execute sequence
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

    # =====================================================================
    #  Vision: camera_optical_link → link_base 
    # =====================================================================
    def get_object_position_base(self, object_name):
        """Request 3D position """
        req = ObjectPosition.Request()
        req.object_name = object_name
        future = self.vision_client.call_async(req)

        # wait the response while spin is on
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
            xyz = [point_base.point.x,
                point_base.point.y,
                point_base.point.z + self.z_offset] 
            self.get_logger().info(f"📍 Object '{object_name}' in base frame (with z_offset): {xyz}")
            return xyz
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None
        
    def compute_ik(self, *args):
        """
        Calculation of the pose
        """
        # 🔹 
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


    # =====================================================================
    #  Arm movement and gripper
    # =====================================================================
    def move_arm(self, positions, duration=3.0):
        if not positions:
            self.get_logger().error("Invalid joint positions, skipping arm motion.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
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

    # =====================================================================
    #  Attach / Detach
    # =====================================================================
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



    # =====================================================================
    #  Sequence of pick & place 
    # =====================================================================

    def pick_and_place(self, model, link, grasp, pre, post, place, rpy_pick, rpy_place):
        self.get_logger().info(f"🦾 Executing pick_and_place for {model}::{link}")

        # -------------------------
        # 🔹 Convert RPY → Quaternion (pick)
        # -------------------------
        qx, qy, qz, qw = quaternion_from_euler(*rpy_pick)

        # Pick pose
        pose_pick = Pose()
        pose_pick.position.x = grasp[0]
        pose_pick.position.y = grasp[1]
        pose_pick.position.z = grasp[2]
        pose_pick.orientation.x = qx
        pose_pick.orientation.y = qy
        pose_pick.orientation.z = qz
        pose_pick.orientation.w = qw

        # -------------------------
        # 🔹 Pre-pick pose
        # -------------------------
        pose_pre = Pose()
        pose_pre.position.x = pre[0]
        pose_pre.position.y = pre[1]
        pose_pre.position.z = pre[2]
        pose_pre.orientation = pose_pick.orientation  # stessa orientazione

        # -------------------------
        # 🔹 Post-pick pose
        # -------------------------
        pose_post = Pose()
        pose_post.position.x = post[0]
        pose_post.position.y = post[1]
        pose_post.position.z = post[2]
        pose_post.orientation = pose_pick.orientation

        # -------------------------
        # 🔹 Place pose
        # -------------------------
        qx, qy, qz, qw = quaternion_from_euler(*rpy_place)
        pose_place = Pose()
        pose_place.position.x = place[0]
        pose_place.position.y = place[1]
        pose_place.position.z = place[2]
        pose_place.orientation.x = qx
        pose_place.orientation.y = qy
        pose_place.orientation.z = qz
        pose_place.orientation.w = qw

        # -------------------------
        # 🔹 Log (debug)
        # -------------------------
        self.get_logger().info(f"📦 pick_rpy={rpy_pick} → quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})")
        self.get_logger().info(f"📦 place_rpy={rpy_place} → quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})")

        # -------------------------
        # 🔹 Movements
        # -------------------------

        # 1️⃣ Pre-pick
        self.get_logger().info("➡️ Moving to pre-pick pose...")
        joint_pre = self.compute_ik(pose_pre)
        if joint_pre: self.move_arm(joint_pre)

        # 2️⃣ Pick
        self.get_logger().info("⬇️ Moving to pick pose...")
        joint_pick = self.compute_ik(pose_pick)
        if joint_pick: self.move_arm(joint_pick)

        # 3️⃣ Close gripper and attach
        self.get_logger().info("✋ Closing gripper and attaching object...")
        self.move_gripper(0.17)
        self.attach_object(model, link)

        # 4️⃣ Post-pick 
        self.get_logger().info("⬆️ Lifting to post-pick pose...")
        joint_post = self.compute_ik(pose_post)
        if joint_post: self.move_arm(joint_post)

        # 5️⃣ Place
        self.get_logger().info("➡️ Moving to place pose...")
        joint_place = self.compute_ik(pose_place)
        if joint_place: self.move_arm(joint_place)

        # 6️⃣ Release
        self.get_logger().info("🖐 Opening gripper and detaching object...")
        self.move_gripper(0.0)
        self.detach_object(model, link)

        # 7️⃣ Back to safe position
        self.get_logger().info("🔁 Returning to post-pick pose...")
        if joint_post: self.move_arm(joint_post)

        self.get_logger().info("✅ pick_and_place completed.")

    
    # =====================================================================
    #  Direct motion from /target_pose  (integrated from move_to_position_llm)
    # =====================================================================
    def pose_callback(self, pose_msg: PoseStamped):
        """Callback when a PoseStamped is received from /target_pose."""
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
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        ordered_positions = [name_to_pos[j] for j in arm_joints if j in name_to_pos]
        self.get_logger().info(f"✅ [LLM] IK solution found: {ordered_positions}")
        self.send_trajectory(arm_joints, ordered_positions)

    def send_trajectory(self, joint_names, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points.append(point)

        self.get_logger().info("🚀 [LLM] Sending trajectory to controller...")
        send_future = self.arm_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected.")
            return
        self.get_logger().info("Trajectory accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("✅ Trajectory execution finished.")
        self.get_logger().info(f"Result error_code: {result.error_code}")



def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceVisionLLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()