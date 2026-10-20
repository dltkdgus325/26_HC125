#!/usr/bin/env python3
"""Own the USB camera and publish frames as ROS2 images."""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    """Publish frames from one directly connected USB camera."""

    def __init__(self):
        """Load camera parameters and open the configured video device."""
        super().__init__('camera_publisher_node')
        defaults = {
            'device': '/dev/video0', 'width': 1280, 'height': 720,
            'fps': 30.0, 'image_topic': '/camera/image_raw',
            'frame_id': 'camera',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def get(name):
            return self.get_parameter(name).value

        self.device = str(get('device'))
        self.width = int(get('width'))
        self.height = int(get('height'))
        self.fps = float(get('fps'))
        self.image_topic = str(get('image_topic'))
        self.frame_id = str(get('frame_id'))
        if self.width <= 0 or self.height <= 0:
            raise ValueError('camera width and height must be > 0')
        if self.fps <= 0.0:
            raise ValueError('camera fps must be > 0')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, self.image_topic, qos_profile_sensor_data
        )
        self.camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.camera.isOpened():
            raise RuntimeError(f'cannot open USB camera: {self.device}')
        actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.camera.get(cv2.CAP_PROP_FPS))
        if actual_fps > 0.0:
            self.fps = actual_fps
        self.last_warning = 0.0
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            f'Camera publisher started: {self.device}, '
            f'{actual_width}x{actual_height}@{self.fps:g} -> '
            f'{self.image_topic}'
        )

    def publish_frame(self):
        """Read and publish one camera frame."""
        success, frame = self.camera.read()
        if not success or frame is None or frame.size == 0:
            now = time.monotonic()
            if now - self.last_warning >= 1.0:
                self.get_logger().warning('USB camera frame read failed')
                self.last_warning = now
            return
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self):
        """Release the USB camera before node destruction."""
        if self.camera.isOpened():
            self.camera.release()
        return super().destroy_node()


def main(args=None):
    """Run the USB camera image publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
