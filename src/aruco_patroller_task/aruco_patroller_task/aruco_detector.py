#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from cv2 import aruco


class ArucoViewerNode(Node):
    def __init__(self):
        super().__init__("aruco_viewer")

        # Parameter for image topic
        self.declare_parameter("image_topic", "/oak/rgb/color")
        image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

        self.get_logger().info(f"Subscribing image: {image_topic}")

        self.bridge = CvBridge()

        # ArUco dictionary + detector params
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 1)

    def image_callback(self, msg: Image):
        # Convert ROS -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        # Draw detections
        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                c = corners[i][0]
                x, y = int(c[0][0]), int(c[0][1])
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

        cv2.imshow("Aruco Debug", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # Clean up OpenCV windows on shutdown
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()