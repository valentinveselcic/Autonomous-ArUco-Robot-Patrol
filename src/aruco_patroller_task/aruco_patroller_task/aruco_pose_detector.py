#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs # Obavezno za rad buffer.transform s PoseStamped

class ArucoPoseDetector(Node):
    def __init__(self):
        super().__init__('aruco_pose_detector')

        # --- PARAMETRI ---
        self.declare_parameter("marker_size", 0.3) 
        self.marker_size = self.get_parameter("marker_size").value
        
        self.declare_parameter("camera_topic", "/oak/rgb/color")
        self.declare_parameter("camera_info_topic", "/oak/rgb/camera_info")
        
        # --- TF BUFFER ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- PUBLISHER ---
        self.marker_pub = self.create_publisher(PoseStamped, '/detected_marker_pose', 10)

        # --- CV SETUP ---
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None


        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        self.create_subscription(CameraInfo, 
                                 self.get_parameter("camera_info_topic").value, 
                                 self.info_callback, 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, 
                                 self.get_parameter("camera_topic").value, 
                                 self.image_callback, qos)

        self.get_logger().info(f"Aruco Detector ready. Searching for markers of size: {self.marker_size}m...")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera Info successfully loaded.")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detekcija
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                # --- PRIPREMA POZE ---
                pose_cam = PoseStamped()
                

                
                pose_cam.header.frame_id = "oak_rgb_camera_optical_frame" 
                

                pose_cam.header.stamp = Time(seconds=0).to_msg() 

                pose_cam.pose.position.x = tvecs[i][0][0]
                pose_cam.pose.position.y = tvecs[i][0][1]
                pose_cam.pose.position.z = tvecs[i][0][2]

                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                q = self.rotation_matrix_to_quaternion(rot_matrix)
                pose_cam.pose.orientation.x = q[0]
                pose_cam.pose.orientation.y = q[1]
                pose_cam.pose.orientation.z = q[2]
                pose_cam.pose.orientation.w = q[3]

                # --- TRANSFORMACIJA ---
                try:
                    if self.tf_buffer.can_transform("map", pose_cam.header.frame_id, Time(), timeout=Duration(seconds=0.1)):
                        
                        pose_map = self.tf_buffer.transform(pose_cam, "map")
                        self.marker_pub.publish(pose_map)
                        
                        
                        if hasattr(cv2, 'drawFrameAxes'):
                            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.2) 
                        else:
                            aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.2)

                except Exception as e:
                    pass

            aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("Aruco View", cv_image)
        cv2.waitKey(1)

    def rotation_matrix_to_quaternion(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()