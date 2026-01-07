#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        # Subscriber alla camera RGB
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription

        cv2.namedWindow("Color Detection")
        cv2.setMouseCallback("Color Detection", self.mouse_callback)
        self.last_frame = None

        self.get_logger().info('🎥 Color detection node started. Waiting for /camera/image_raw...')

    # -----------------------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_frame = frame.copy()

        # Converti in HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- 🔴 RED
        lower_red1 = np.array([0, 180, 50])
        upper_red1 = np.array([10, 255, 180])
        lower_red2 = np.array([170, 180, 50])
        upper_red2 = np.array([180, 255, 180])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        kernel = np.ones((5,5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)


        # --- 🟢 GREEN ---
        lower_green = np.array([70, 150, 60])
        upper_green = np.array([95, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Detect e disegna bounding box
        self.detect_and_draw(frame, mask_red, (0, 0, 255), "Red Cube")
        self.detect_and_draw(frame, mask_green, (0, 255, 0), "Green Cylinder")

        # Visualizza
        cv2.imshow("Color Detection", frame)
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Green Mask", mask_green)
        cv2.waitKey(1)

    # -----------------------------
    def detect_and_draw(self, frame, mask, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 200:
            return

        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w // 2, y + h // 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
        cv2.putText(frame, f'{label} ({cx},{cy})', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # -----------------------------
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.last_frame is not None:
            bgr = self.last_frame[y, x].tolist()
            hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0].tolist()
            self.get_logger().info(f'Pixel @({x},{y}) → BGR={bgr}, HSV={hsv}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
