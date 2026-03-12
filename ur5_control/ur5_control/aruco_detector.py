#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
ArUco Marker Detection and TF Publishing Node

Detects ArUco markers in the RealSense D435 camera feed, estimates their
3D pose using depth data, and publishes TF transforms so the UR5 arm
can plan collision-free pick trajectories to each detected package.

Pipeline:
  1. Subscribe to RGB + aligned depth streams from RealSense
  2. Detect 4x4_50 ArUco markers, filter by area threshold
  3. Estimate 6-DOF pose via solvePnP (cv2.aruco.estimatePoseSingleMarkers)
  4. Fuse with aligned depth to get metric 3D position in camera frame
  5. Publish camera_link -> cam_<id> transform
  6. Lookup base_link -> cam_<id> via TF tree, republish as obj_<id>

Author: Parth Khanna
"""

import rclpy
import sys
import cv2
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

# RealSense D435 camera intrinsics (from Gazebo plugin /camera_info)
CAM_MAT = np.array([[931.1829833984375, 0.0, 640.0],
                     [0.0, 931.1829833984375, 360.0],
                     [0.0, 0.0, 1.0]])
DIST_MAT = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

ARUCO_SIZE_M = 0.15         # 150mm physical marker size
ARUCO_AREA_THRESHOLD = 1500 # pixel area — filters out far/small markers

# Image dimensions and focal lengths
SIZE_X, SIZE_Y = 1280, 720
CENTER_X, CENTER_Y = 640, 360
FOCAL_X, FOCAL_Y = 931.1829833984375, 931.1829833984375


def calculate_rectangle_area(coordinates):
    """Compute pixel area and width of an ArUco marker from its 4 corners."""
    coord = coordinates.reshape(4, 2)
    width = np.linalg.norm(coord[0] - coord[1])
    height = np.linalg.norm(coord[1] - coord[2])
    return width * height, width


def detect_aruco(image):
    """
    Detect ArUco markers and estimate their pose.

    Returns lists of: center pixels, distances, yaw angles, widths, and marker IDs.
    Only markers with pixel area above ARUCO_AREA_THRESHOLD are returned.
    """
    center_list, distance_list, angle_list, width_list, id_list = [], [], [], [], []

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    corners, detected_ids, _ = detector.detectMarkers(gray)

    if detected_ids is None:
        return center_list, distance_list, angle_list, width_list, id_list

    cv2.aruco.drawDetectedMarkers(image, corners, detected_ids)

    for i, marker_id in enumerate(detected_ids):
        corner = corners[i][0]
        area, width = calculate_rectangle_area(corner)

        if area < ARUCO_AREA_THRESHOLD:
            continue

        cx = int(np.mean(corner[:, 0]))
        cy = int(np.mean(corner[:, 1]))
        center_list.append((cx, cy))

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[i:i+1], ARUCO_SIZE_M, CAM_MAT, DIST_MAT)
        distance_list.append(tvecs[0][0][2])

        rot_matrix, _ = cv2.Rodrigues(rvecs[0][0])
        euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=False)
        angle_list.append(euler[2])

        width_list.append(width)
        id_list.append(marker_id[0])

        cv2.drawFrameAxes(image, CAM_MAT, DIST_MAT, rvecs[0], tvecs[0], 0.1)

    return center_list, distance_list, angle_list, width_list, id_list


class ArucoTFPublisher(Node):
    """
    ROS 2 node that detects ArUco markers and broadcasts their pose as TF frames.

    Subscribes to:
        /camera/color/image_raw                     — RGB image
        /camera/aligned_depth_to_color/image_raw    — aligned depth image

    Publishes:
        /tf  — cam_<id> (w.r.t. camera_link) and obj_<id> (w.r.t. base_link)
    """

    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.color_cam_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._color_cb, 10)
        self.depth_cam_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Process images at 5 Hz
        self.timer = self.create_timer(0.2, self._process)

        self.cv_image = None
        self.depth_image = None

    def _depth_cb(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def _color_cb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def _process(self):
        if self.cv_image is None or self.depth_image is None:
            return

        image = self.cv_image.copy()
        centers, distances, angles, widths, ids = detect_aruco(image)

        for i, marker_id in enumerate(ids):
            cX, cY = centers[i]

            # Angle correction (empirical calibration for Gazebo ArUco orientation)
            angle = angles[i]
            angle_corrected = (0.788 * angle) - ((angle ** 2) / 3160.0)

            quat = R.from_euler('xyz', [0.0, 0.0, angle_corrected]).as_quat()

            # Depth from aligned RealSense stream (mm -> m), fallback to pose estimation
            depth = self.depth_image[cY, cX] / 1000.0
            if depth <= 0:
                depth = distances[i]

            self.get_logger().info(f'Marker {marker_id}: depth = {depth:.3f} m')

            # Pinhole camera model: pixel + depth -> 3D point in camera frame
            x = depth * (SIZE_X - cX - CENTER_X) / FOCAL_X
            y = depth * (SIZE_Y - cY - CENTER_Y) / FOCAL_Y
            z = depth

            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            # Broadcast camera_link -> cam_<id>
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

            # Lookup base_link -> cam_<id> and republish as obj_<id> for arm planning
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
    node = ArucoTFPublisher()
    node.get_logger().info('ArUco TF publisher started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
