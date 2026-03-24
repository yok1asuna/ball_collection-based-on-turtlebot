#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import message_filters
import numpy as np
from ultralytics import YOLO

class RgbdYoloNode(Node):
    def __init__(self):
        super().__init__('rgbd_yolo_node')
        self.bridge = CvBridge()
        
        # 加载 YOLO 模型 (替换 best.pt)
        self.model = YOLO("yolo11n.pt") 
        
        #相机内参 (通过订阅 /camera/camera_info 动态获取)
        self.camera_info_received = False

        #订阅相机内参
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        #创建两个独立的订阅器
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_rect_raw')

        #对齐彩色流与深度流 
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        self.target_pose_pub = self.create_publisher(PoseArray, '/vision/target_poses', 10)
        self.get_logger().info("RGB-D YOLO 时间对齐节点已启动！")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]  # fx
            self.fy = msg.k[4]  # fy
            self.cx = msg.k[2]  # cx
            self.cy = msg.k[5]  # cy
            self.camera_info_received = True
            self.get_logger().info(f"相机内参已更新: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def sync_callback(self, color_msg, depth_msg):
        # Validate camera info received before processing
        if not self.camera_info_received:
            self.get_logger().warn("Camera info not received yet, skipping frame")
            return
        
        # Convert ROS images to numpy matrices
        cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        # Perform YOLO inference on the color image
        results = self.model(cv_color, verbose=False)

        points = []
        for r in results:
            for box in r.boxes:
                if float(box.conf[0]) < 0.5: 
                    continue

                # Obtain the center of the pixel (u, v)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = int((x1 + x2) / 2.0)
                v = int((y1 + y2) / 2.0)

                # Clip to image bounds
                u = int(np.clip(u, 0, cv_depth.shape[1] - 1))
                v = int(np.clip(v, 0, cv_depth.shape[0] - 1))
                
                depth_mm = int(cv_depth[v, u])
                if depth_mm == 0: 
                    continue  # 0 表示深度相机在该点存在盲区或反光，舍弃
                
                Z_c = depth_mm / 1000.0 

                # 针孔相机模型反投影
                X_c = (u - self.cx) * Z_c / self.fx
                Y_c = (v - self.cy) * Z_c / self.fy

                points.append([X_c, Y_c, Z_c])

        if points:
            self.publish_target_poses(points, color_msg.header)
        else:
            self.get_logger().debug("No valid ball detections in this frame")

    def publish_target_poses(self, points, header):
        if not points or len(points) == 0:
            self.get_logger().warn("Cannot publish empty target pose array")
            return

        pose_array = PoseArray()
        pose_array.header = header

        for p in points:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.target_pose_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(points)} target poses")

def main(args=None):
    rclpy.init(args=args)
    node = RgbdYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()