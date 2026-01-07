#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectPositionNode(Node):
    def __init__(self):
        super().__init__('object_position_node')
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)

        # State variables
        self.rgb_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info('🎯 Object position node started — waiting for camera topics...')

    # --------------------------
    def info_callback(self, msg):
        """Save intrinsic parameter of the camera."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().debug(f'Camera intrinsics loaded: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}')

    # --------------------------

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # --------------------------
    def rgb_callback(self, msg):
        """GET RGB and calculate the 3D position based on colored mask"""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.depth_image is None or self.fx is None:
            return

        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        # --- CALIBRATED MASK ---
        lower_red1 = np.array([0, 180, 50])
        upper_red1 = np.array([10, 255, 180])
        lower_red2 = np.array([170, 180, 50])
        upper_red2 = np.array([180, 255, 180])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        lower_green = np.array([70, 150, 60])
        upper_green = np.array([95, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Get centroid of the mask
        self.process_mask(mask_red, (0, 0, 255), 'Red Cube')
        self.process_mask(mask_green, (0, 255, 0), 'Green Cylinder')

        cv2.imshow('RGB', self.rgb_image)
        cv2.waitKey(1)

    # --------------------------
    def process_mask(self, mask, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 100:
            return

        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w // 2, y + h // 2
        cv2.rectangle(self.rgb_image, (x, y), (x + w, y + h), color, 2)
        cv2.circle(self.rgb_image, (cx, cy), 5, color, -1)

        # Get the depth of the point
        Z = float(self.depth_image[cy, cx])
        if Z == 0.0 or np.isnan(Z):
            self.get_logger().warn(f'No valid depth at ({cx},{cy})')
            return

        # CONVERSION from PIXEL TO METERS
        X = (cx - self.cx) * Z / self.fx
        Y = (cy - self.cy) * Z / self.fy

        text = f'{label}: X={X:.3f} Y={Y:.3f} Z={Z:.3f} m'
        cv2.putText(self.rgb_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.get_logger().info(text)



def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
