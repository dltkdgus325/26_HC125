#!/usr/bin/env python3
"""Capture USB-camera photographs and videos through ROS services."""

import threading
import select
import sys
import termios
import tty
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class CameraRecorder(Node):
    """Record and capture images received from a ROS2 camera topic."""

    def __init__(self):
        """Subscribe to camera images and create recording services."""
        super().__init__('camera_recorder')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('video_codec', 'mp4v')
        self.declare_parameter(
            'output_directory', '/home/rodin/boat_data/camera'
        )

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.jpeg_quality = int(
            self.get_parameter('jpeg_quality').value
        )
        self.video_codec = str(
            self.get_parameter('video_codec').value
        )
        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        self.image_directory = self.output_directory / 'images'
        self.video_directory = self.output_directory / 'videos'
        self.image_directory.mkdir(parents=True, exist_ok=True)
        self.video_directory.mkdir(parents=True, exist_ok=True)
        self._validate_parameters()

        self.bridge = CvBridge()
        self.frame_size = (self.width, self.height)
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.video_writer = None
        self.recording_path = None
        self.keyboard_stop = threading.Event()
        self.capture_service = self.create_service(
            Trigger, '/camera/capture', self.capture_image
        )
        self.start_service = self.create_service(
            Trigger, '/camera/start_recording', self.start_recording
        )
        self.stop_service = self.create_service(
            Trigger, '/camera/stop_recording', self.stop_recording
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback,
            qos_profile_sensor_data
        )
        self.keyboard_thread = None
        if sys.stdin.isatty():
            self.keyboard_thread = threading.Thread(
                target=self.keyboard_loop,
                daemon=True,
            )
            self.keyboard_thread.start()
        else:
            self.get_logger().warning(
                'No interactive terminal: keyboard control is disabled'
            )
        self.get_logger().info(
            f'Camera recorder subscribed: {self.image_topic}, '
            f'recording at {self.fps:g} fps'
        )

    def _validate_parameters(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError('camera width and height must be > 0')
        if self.fps <= 0.0:
            raise ValueError('camera fps must be > 0')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        if len(self.video_codec) != 4:
            raise ValueError(
                'video_codec must contain exactly four characters'
            )

    def image_callback(self, message):
        """Store one ROS image and append it when recording is active."""
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().warning(f'CvBridge conversion failed: {exc}')
            return
        with self.frame_lock:
            self.latest_frame = frame
            if self.video_writer is None:
                self.frame_size = (frame.shape[1], frame.shape[0])
            if self.video_writer is not None:
                self.video_writer.write(frame)

    def capture_image(self, request, response):
        """Save the most recently captured frame as JPEG."""
        del request
        success, message = self._save_image()
        response.success = success
        response.message = message
        return response

    def _save_image(self):
        """Save the latest frame and return success and status text."""
        with self.frame_lock:
            if self.latest_frame is None:
                return False, 'camera frame is not ready'
            frame = self.latest_frame.copy()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        image_path = self.image_directory / f'{timestamp}.jpg'
        success = cv2.imwrite(
            str(image_path),
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if success:
            self.get_logger().info(f'Photo saved: {image_path}')
            return True, str(image_path)
        return False, 'failed to write image'

    def start_recording(self, request, response):
        """Open a new video file and start writing captured frames."""
        del request
        success, message = self._start_recording()
        response.success = success
        response.message = message
        return response

    def _start_recording(self):
        """Start a video file and return success and status text."""
        with self.frame_lock:
            if self.video_writer is not None:
                return False, f'already recording: {self.recording_path}'
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            video_path = self.video_directory / f'{timestamp}.mp4'
            fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
            writer = cv2.VideoWriter(
                str(video_path), fourcc, self.fps, self.frame_size
            )
            if not writer.isOpened():
                return False, 'failed to open video writer'
            self.video_writer = writer
            self.recording_path = video_path
        self.get_logger().info(f'Recording started: {video_path}')
        return True, str(video_path)

    def stop_recording(self, request, response):
        """Stop recording and close the current video file."""
        del request
        success, message = self._stop_recording()
        response.success = success
        response.message = message
        return response

    def _stop_recording(self):
        """Stop the active video file and return success and status text."""
        with self.frame_lock:
            if self.video_writer is None:
                return False, 'recording is not active'
            video_path = self.recording_path
            self.video_writer.release()
            self.video_writer = None
            self.recording_path = None
        self.get_logger().info(f'Recording stopped: {video_path}')
        return True, str(video_path)

    def keyboard_loop(self):
        """Read photo and recording keys from the attached SSH terminal."""
        old_settings = termios.tcgetattr(sys.stdin)
        print(
            '카메라 제어: [p] 사진  [r] 녹화 시작/종료  '
            '[s] 녹화 종료  [q] 카메라 종료',
            flush=True,
        )
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not self.keyboard_stop.is_set() and rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue
                key = sys.stdin.read(1).lower()
                if key == 'p':
                    success, message = self._save_image()
                elif key == 'r':
                    with self.frame_lock:
                        is_recording = self.video_writer is not None
                    if is_recording:
                        success, message = self._stop_recording()
                    else:
                        success, message = self._start_recording()
                elif key == 's':
                    success, message = self._stop_recording()
                elif key == 'q':
                    self.keyboard_stop.set()
                    rclpy.try_shutdown()
                    break
                else:
                    continue
                status = '성공' if success else '실패'
                print(f'{status}: {message}', flush=True)
        finally:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, old_settings
            )

    def destroy_node(self):
        """Release the video writer before destruction."""
        self.keyboard_stop.set()
        if (
            self.keyboard_thread is not None
            and self.keyboard_thread is not threading.current_thread()
        ):
            self.keyboard_thread.join(timeout=1.0)
        with self.frame_lock:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
        return super().destroy_node()


def main(args=None):
    """Run the ROS image recorder node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraRecorder()
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
