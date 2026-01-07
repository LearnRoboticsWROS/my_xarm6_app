#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from my_xarm6_app.llm.ollama_client import OllamaClient
import json
import tf_transformations as tf

class LLMCommandNode(Node):
    def __init__(self):
        super().__init__('llm_command_node')
        self.sub = self.create_subscription(String, 'llm_command', self.command_callback, 10)
        self.pub = self.create_publisher(PoseStamped, 'target_pose', 10)

        self.ollama = OllamaClient(model='llama3')

        self.system_prompt = (
            "You are a robotics assistant that outputs ONLY valid JSON. "
            "Do not include any text or markdown. Example format:\n"
            '{"frame":"link_base","position":{"x":0.2,"y":0.1,"z":0.3},'
            '"orientation_rpy":{"roll":0.0,"pitch":1.57,"yaw":0.0}}'
        )


        self.get_logger().info('🧠 LLM Command Node ready. Publish a string to /llm_command.')

    def command_callback(self, msg):
        user_cmd = msg.data
        self.get_logger().info(f'Received LLM command: {user_cmd}')
        
        try:
            response = self.ollama.chat(self.system_prompt, user_cmd).strip()
            if not response:
                self.get_logger().error('❌ LLM returned empty response.')
                return

            if response.startswith("```"):
                response = response.strip('`')
                response = response.replace("json", "").strip()

            self.get_logger().info(f'💬 Raw LLM reply: {response}')
            pose_data = json.loads(response)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ JSON parsing failed: {e}')
            self.get_logger().error(f'Raw response was: {response}')
            return

        pos = pose_data["position"]
        rpy = pose_data["orientation_rpy"]

        roll = float(rpy["roll"])
        pitch = float(rpy["pitch"])
        yaw = float(rpy["yaw"])
        q = tf.quaternion_from_euler(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header.frame_id = pose_data.get("frame", "link_base")
        pose.pose.position.x = float(pos["x"])
        pose.pose.position.y = float(pos["y"])
        pose.pose.position.z = float(pos["z"])

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pub.publish(pose)
        self.get_logger().info('📤 Published PoseStamped to /target_pose.')





def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()