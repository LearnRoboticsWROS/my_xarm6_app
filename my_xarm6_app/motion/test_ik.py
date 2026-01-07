#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from tf_transformations import quaternion_from_euler


class TestIKClient(Node):
    """
    Simple IK test node for xarm6 using MoveIt GetPositionIK service.
    """

    def __init__(self):
        super().__init__('test_ik_client')

        # Name of service MoveIt per IK (di default)
        self.service_name = 'compute_ik'

        self.get_logger().info(f'Waiting for IK service: {self.service_name} ...')
        self.cli = self.create_client(GetPositionIK, self.service_name)

        # Wait the service  availability
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service {self.service_name} not available!')
            raise RuntimeError('IK service not available')

        self.get_logger().info(f'Service {self.service_name} available, sending IK request...')

    
    def send_ik_request(self):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'

        # Pose target 
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Orientation: 
        qx, qy, qz, qw = quaternion_from_euler(-2.292, 0.0, -1.57)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # Position target
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.5
        pose.pose.position.z = 0.5

        req.ik_request.pose_stamped = pose

        # Opzional
        req.ik_request.avoid_collisions = True

        # Async call to the service
        future = self.cli.call_async(req)
        return future
    
    def process_result(self, res: GetPositionIK.Response):
        if res.error_code.val == res.error_code.SUCCESS:
            self.get_logger().info('IK SUCCESS, solution found.')

            js = res.solution.joint_state
            # print joint name
            for name, pos in zip(js.name, js.position):
                self.get_logger().info(f'  {name}: {pos:.4f}')
        else:
            self.get_logger().error(f'IK FAILED, error code: {res.error_code.val}')
    


def main(args=None):
    rclpy.init(args=args)
    node = TestIKClient()

    future = node.send_ik_request()

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.process_result(future.result())
    else:
        node.get_logger().error('No result from IK service!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()