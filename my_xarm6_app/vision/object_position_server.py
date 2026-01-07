#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from my_xarm6_interfaces.srv import ObjectPosition


class ObjectPositionServer(Node):
    def __init__(self):
        super().__init__('object_position_server')
        self.bridge = CvBridge()

        # Camera topics
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)

        self.service = self.create_service(ObjectPosition, 'get_object_position', self.handle_request)

        self.rgb_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info('Object position server ready (waiting for images)...')

    # --------------------------
    def info_callback(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    # --------------------------
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # --------------------------
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # --------------------------
    def handle_request(self, request, response):
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn('No camera frames yet.')
            response.success = False
            return response

        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        if request.object_name.lower() == 'red':
            mask = self.get_red_mask(hsv)
        elif request.object_name.lower() == 'green':
            mask = self.get_green_mask(hsv)
        else:
            self.get_logger().error(f'Unknown object name: {request.object_name}')
            response.success = False
            return response

        c = self.get_largest_contour(mask)
        if c is None:
            response.success = False
            return response

        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w // 2, y + h // 2
        Z = float(self.depth_image[cy, cx])

        if Z == 0.0 or np.isnan(Z):
            self.get_logger().warn(f'Invalid depth for {request.object_name}.')
            response.success = False
            return response

        X = (cx - self.cx) * Z / self.fx
        Y = (cy - self.cy) * Z / self.fy

        self.get_logger().info(f'{request.object_name} → X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}')
        response.x, response.y, response.z = X, Y, Z
        response.success = True
        return response

    # --------------------------
    def get_red_mask(self, hsv):
        lower_red1 = np.array([0, 180, 50])
        upper_red1 = np.array([10, 255, 180])
        lower_red2 = np.array([170, 180, 50])
        upper_red2 = np.array([180, 255, 180])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        return mask

    def get_green_mask(self, hsv):
        lower_green = np.array([70, 150, 60])
        upper_green = np.array([95, 255, 255])
        return cv2.inRange(hsv, lower_green, upper_green)

    def get_largest_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            self.get_logger().warn('No contours detected.')
            return None
        return max(contours, key=cv2.contourArea)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()