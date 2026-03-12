#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
ArUco marker detection and TF publishing for warehouse pick-and-place.

Detects ArUco markers using the RealSense depth camera, estimates their
3D pose relative to camera_link, then publishes the transform w.r.t.
base_link so the UR5 arm can plan pick trajectories.
"""

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image


# Camera intrinsics (from RealSense D435 Gazebo plugin)
CAM_MAT = np.array([[931.1829833984375, 0.0, 640.0],
                     [0.0, 931.1829833984375, 360.0],
                     [0.0, 0.0, 1.0]])
DIST_MAT = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
ARUCO_SIZE = 0.15           # 150mm markers
ARUCO_AREA_THRESHOLD = 1500 # ignore small/far markers

SIZE_CAM_X = 1280
SIZE_CAM_Y = 720
CENTER_CAM_X = 640
CENTER_CAM_Y = 360
FOCAL_X = 931.1829833984375
FOCAL_Y = 931.1829833984375


def calculate_rectangle_area(coordinates):
    """Calculate area and width of detected ArUco marker from corner coords."""
    coord = coordinates.reshape(4, 2)
    width = np.linalg.norm(coord[0] - coord[1])
    height = np.linalg.norm(coord[1] - coord[2])
    area = width * height
    return area, width


def detect_aruco(image):
    """
    Detect ArUco markers and return center points, distances, angles, widths, and IDs.
    Only returns markers within the arm's reach (area > threshold).
    """
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids_list = []

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_list

    for i, marker_id in enumerate(ids):
        corner = corners[i][0]
        area, width = calculate_rectangle_area(corner)

        if area < ARUCO_AREA_THRESHOLD:
            continue

        # Center point
        cx = int(np.mean(corner[:, 0]))
        cy = int(np.mean(corner[:, 1]))
        center_aruco_list.append((cx, cy))

        # Pose estimation
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[i:i+1], ARUCO_SIZE, CAM_MAT, DIST_MAT)
        distance = tvecs[0][0][2]
        distance_from_rgb_list.append(distance)

        # Extract yaw angle from rotation vector
        rot_matrix, _ = cv2.Rodrigues(rvecs[0][0])
        r = R.from_matrix(rot_matrix)
        euler = r.as_euler('xyz', degrees=False)
        angle_aruco_list.append(euler[2])

        width_aruco_list.append(width)
        ids_list.append(marker_id[0])

        # Draw marker and axes on image
        cv2.aruco.drawDetectedMarkers(image, corners[i:i+1], ids[i:i+1])
        cv2.drawFrameAxes(image, CAM_MAT, DIST_MAT, rvecs[0], tvecs[0], 0.1)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_list


class ArucoTF(Node):
    """Detects ArUco markers and publishes TF transforms for pick-and-place."""

    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.color_cam_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.process_image)

        self.cv_image = None
        self.depth_image = None

    def depthimagecb(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def colorimagecb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def process_image(self):
        if self.cv_image is None or self.depth_image is None:
            return

        image = self.cv_image.copy()
        centers, distances, angles, widths, ids = detect_aruco(image)

        for i, marker_id in enumerate(ids):
            cX, cY = centers[i]

            # Get depth from aligned depth image (mm -> m)
            depth = self.depth_image[cY, cX] / 1000.0
            if depth <= 0:
                depth = distances[i]

            # Correct aruco angle using calibration formula
            angle = angles[i]
            angle_corrected = (0.788 * angle) - ((angle ** 2) / 3160.0)

            # Convert pixel + depth to 3D coordinates in camera frame
            x = depth * (SIZE_CAM_X - cX - CENTER_CAM_X) / FOCAL_X
            y = depth * (SIZE_CAM_Y - cY - CENTER_CAM_Y) / FOCAL_Y
            z = depth

            # Quaternion from corrected yaw (roll=0, pitch=0)
            quat = R.from_euler('xyz', [0.0, 0.0, angle_corrected]).as_quat()

            # Mark center on image
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            # Publish TF: camera_link -> cam_<id>
            t_cam = TransformStamped()
            t_cam.header.stamp = self.get_clock().now().to_msg()
            t_cam.header.frame_id = 'camera_link'
            t_cam.child_frame_id = f'cam_{marker_id}'
            t_cam.transform.translation.x = x
            t_cam.transform.translation.y = y
            t_cam.transform.translation.z = z
            t_cam.transform.rotation.x = quat[0]
            t_cam.transform.rotation.y = quat[1]
            t_cam.transform.rotation.z = quat[2]
            t_cam.transform.rotation.w = quat[3]
            self.br.sendTransform(t_cam)

            # Lookup base_link -> cam_<id> and republish as obj_<id>
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', f'cam_{marker_id}', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))

                t_obj = TransformStamped()
                t_obj.header.stamp = self.get_clock().now().to_msg()
                t_obj.header.frame_id = 'base_link'
                t_obj.child_frame_id = f'obj_{marker_id}'
                t_obj.transform = trans.transform
                self.br.sendTransform(t_obj)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed for marker {marker_id}: {e}')

        cv2.imshow('ArUco Detection', image)
        cv2.waitKey(1)


def main():
    rclpy.init(args=sys.argv)
    node = ArucoTF()
    node.get_logger().info('ArUco TF publisher started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
