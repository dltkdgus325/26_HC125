#!/usr/bin/env python3
"""Detect and temporally confirm trash from a ROS2 image stream."""

from __future__ import annotations

import math
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, String

from .target_filter import TargetFilter, draw_filter_overlay
from .target_filter import parse_yolo_detections

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class TrashDetectionNode(Node):
    """Run YOLO tracking and publish only temporally confirmed targets."""

    def __init__(self):
        """Load parameters, model, ROS interfaces, and target filter."""
        super().__init__('trash_detection_node')
        defaults = {
            'model': '/home/rodin/boat_data/best.pt',
            'device': '0', 'detection_conf': 0.30,
            'target_entry_conf': 0.50, 'target_keep_conf': 0.30,
            'confirm_window': 5, 'confirm_count': 3,
            'target_lost_timeout': 0.50,
            'max_target_jump_ratio': 0.20,
            'target_classes': '', 'use_tracking': True, 'imgsz': 640,
            'frame_stride': 1, 'image_topic': '/camera/image_raw',
            'target_topic': '/trash/target',
            'target_state_topic': '/trash/target_state',
            'debug_image_topic': '/trash/debug_image',
            'system_event_topic': '/autonomy/system_event',
            'publish_debug_image': True, 'show_window': False,
            'save_hard_negatives': False,
            'hard_negative_dir': '/home/rodin/boat_data/hard_negatives',
            'hard_negative_save_interval': 1.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def value(name):
            return self.get_parameter(name).value

        self.model_path = Path(str(value('model'))).expanduser()
        self.device = str(value('device'))
        self.detection_conf = float(value('detection_conf'))
        self.target_entry_conf = float(value('target_entry_conf'))
        self.target_keep_conf = float(value('target_keep_conf'))
        self.confirm_window = int(value('confirm_window'))
        self.confirm_count = int(value('confirm_count'))
        self.target_lost_timeout = float(value('target_lost_timeout'))
        self.max_target_jump_ratio = float(value('max_target_jump_ratio'))
        self.target_classes = {
            item.strip() for item in str(value('target_classes')).split(',')
            if item.strip()
        }
        self.use_tracking = bool(value('use_tracking'))
        self.imgsz = int(value('imgsz'))
        self.frame_stride = max(1, int(value('frame_stride')))
        self.publish_debug_image = bool(value('publish_debug_image'))
        self.show_window = bool(value('show_window'))
        self.save_hard_negatives = bool(value('save_hard_negatives'))
        self.hard_negative_dir = Path(
            str(value('hard_negative_dir'))
        ).expanduser()
        self.hard_negative_save_interval = float(
            value('hard_negative_save_interval')
        )
        self._validate_parameters()

        self.bridge = CvBridge()
        self.frame_count = 0
        self.inference_device_checked = False
        self.last_inference_error_event = float('-inf')
        self.last_hard_negative_save = float('-inf')
        self.candidate_frame = None
        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.system_event_pub = self.create_publisher(
            String, str(value('system_event_topic')), event_qos
        )
        self._check_cuda_environment()
        self.yolo = self._load_model()
        self.filter = TargetFilter(
            self.target_entry_conf, self.target_keep_conf,
            self.confirm_window, self.confirm_count,
            self.target_lost_timeout, self.max_target_jump_ratio,
            logger=self.get_logger().info,
        )
        if self.save_hard_negatives:
            self.hard_negative_dir.mkdir(parents=True, exist_ok=True)
        self.target_pub = self.create_publisher(
            Float64MultiArray, str(value('target_topic')), 10
        )
        self.target_state_pub = self.create_publisher(
            String, str(value('target_state_topic')), 10
        )
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(
                Image, str(value('debug_image_topic')), 1
            )
        self.image_sub = self.create_subscription(
            Image, str(value('image_topic')), self.image_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Trash image subscriber started: {value("image_topic")}'
        )

    def _validate_parameters(self):
        for name in ('detection_conf', 'target_entry_conf',
                     'target_keep_conf', 'max_target_jump_ratio'):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f'{name} must be in [0, 1]')
        if self.target_keep_conf < self.detection_conf:
            raise ValueError('target_keep_conf must be >= detection_conf')
        if self.target_entry_conf < self.target_keep_conf:
            raise ValueError('target_entry_conf must be >= target_keep_conf')
        if self.confirm_window <= 0:
            raise ValueError('confirm_window must be > 0')
        if not 1 <= self.confirm_count <= self.confirm_window:
            raise ValueError('confirm_count must be in [1, confirm_window]')
        if (not math.isfinite(self.target_lost_timeout)
                or self.target_lost_timeout <= 0.0):
            raise ValueError('target_lost_timeout must be finite and > 0')
        if self.hard_negative_save_interval < 0.0:
            raise ValueError('hard_negative_save_interval must be >= 0')
        if self.imgsz <= 0:
            raise ValueError('imgsz must be > 0')

    def _load_model(self):
        if not self.model_path.is_file():
            self._emit_system_event(
                'ERROR', f'Model not found: {self.model_path}'
            )
            return None
        if YOLO is None:
            self._emit_system_event('ERROR', 'ultralytics unavailable')
            return None
        try:
            model = YOLO(str(self.model_path))
            try:
                model.fuse()
            except Exception as exc:
                self.get_logger().warning(f'YOLO fuse skipped: {exc}')
            self.get_logger().info(f'YOLO loaded: {self.model_path}')
            self.get_logger().info(f'YOLO classes: {model.names}')
            return model
        except Exception as exc:
            self._emit_system_event('ERROR', f'Model load failed: {exc}')
            return None

    def _gpu_requested(self):
        return self.device.strip().lower() not in ('cpu', '-1')

    def _check_cuda_environment(self):
        """Report CUDA status without preventing node startup."""
        if not self._gpu_requested():
            self._emit_system_event(
                'WARNING', f'YOLO configured for non-CUDA device={self.device}'
            )
            return
        try:
            import torch
            if not torch.cuda.is_available():
                self._emit_system_event(
                    'ERROR', 'CUDA unavailable; YOLO GPU inference may fail'
                )
                return
            gpu = torch.cuda.get_device_name(0)
            self.get_logger().info(
                f'CUDA available: True, GPU: {gpu}, requested device: '
                f'{self.device}'
            )
            if 'orin' not in gpu.lower():
                self._emit_system_event(
                    'WARNING', f'CUDA device is not identified as Orin: {gpu}'
                )
        except Exception as exc:
            self._emit_system_event(
                'ERROR', f'CUDA environment check failed: {exc}'
            )

    def _emit_system_event(self, level, message):
        text = f'{level}|trash_detection|{message}'
        log = self.get_logger().error if level == 'ERROR' \
            else self.get_logger().warning
        log(message)
        event = String()
        event.data = text
        self.system_event_pub.publish(event)

    def image_callback(self, msg):
        """Infer, filter, publish, and optionally visualize one ROS image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'CvBridge conversion failed: {exc}')
            return
        self.frame_count += 1
        if (self.frame_count - 1) % self.frame_stride:
            return
        height, width = frame.shape[:2]
        detections = []
        if self.yolo is not None:
            try:
                inference = (
                    self.yolo.track if self.use_tracking else self.yolo.predict
                )
                kwargs = dict(
                    source=frame, conf=self.detection_conf,
                    device=self.device, imgsz=self.imgsz, verbose=False
                )
                if self.use_tracking:
                    kwargs['persist'] = True
                results = inference(**kwargs)
                if results:
                    self._verify_inference_device()
                    detections = parse_yolo_detections(
                        results[0], self.yolo.names, self.target_classes,
                        width, height, self.get_logger().warning
                    )
            except Exception as exc:
                self.get_logger().error(f'YOLO inference failed: {exc}')
                error_now = time.monotonic()
                if error_now - self.last_inference_error_event >= 1.0:
                    self._emit_system_event(
                        'ERROR', f'YOLO inference failed: {exc}'
                    )
                    self.last_inference_error_event = error_now
        now = time.monotonic()
        filtered = self.filter.update(detections, now)
        state_msg = String()
        state_msg.data = filtered.state
        self.target_state_pub.publish(state_msg)
        if self.save_hard_negatives and filtered.candidate is not None:
            self.candidate_frame = frame.copy()
        if filtered.hard_negative:
            self._save_hard_negative(now)
        if filtered.target is None:
            self.publish_no_target(width, height)
        else:
            self.publish_target(filtered.target)
        self.publish_debug(frame, detections, filtered, msg)

    def _verify_inference_device(self):
        if self.inference_device_checked:
            return
        self.inference_device_checked = True
        try:
            actual = str(self.yolo.predictor.model.device)
        except Exception as exc:
            self._emit_system_event(
                'WARNING', f'Cannot verify YOLO inference device: {exc}'
            )
            return
        self.get_logger().info(f'YOLO inference device: {actual}')
        if self._gpu_requested() and not actual.startswith('cuda'):
            self._emit_system_event(
                'ERROR', f'YOLO is not using CUDA: actual device={actual}'
            )

    def _save_hard_negative(self, now):
        if (not self.save_hard_negatives or self.candidate_frame is None
                or now - self.last_hard_negative_save
                < self.hard_negative_save_interval):
            self.candidate_frame = None
            return
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        millis = int((time.time() % 1.0) * 1000)
        path = self.hard_negative_dir / f'{timestamp}_{millis:03d}.jpg'
        if cv2.imwrite(str(path), self.candidate_frame):
            self.last_hard_negative_save = now
            self.get_logger().info(f'[TRASH] hard negative saved: {path}')
        self.candidate_frame = None

    def publish_target(self, target):
        """Publish a confirmed or maintained target."""
        msg = Float64MultiArray()
        msg.data = [1.0, float(target.class_id), target.confidence,
                    target.center_x, target.center_y, target.bbox_width,
                    target.bbox_height, float(target.image_width),
                    float(target.image_height), float(target.track_id)]
        self.target_pub.publish(msg)

    def publish_no_target(self, width, height):
        """Publish an explicit unconfirmed/no-target result."""
        msg = Float64MultiArray()
        msg.data = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    float(width), float(height), -1.0]
        self.target_pub.publish(msg)

    def publish_debug(self, frame, detections, filtered, source_msg):
        """Publish or display raw and filtered detection annotations."""
        if self.debug_pub is None and not self.show_window:
            return
        debug_frame = draw_filter_overlay(frame, detections, filtered)
        if self.debug_pub is not None:
            msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            msg.header = source_msg.header
            self.debug_pub.publish(msg)
        if self.show_window:
            cv2.imshow('Trash detection', debug_frame)
            cv2.waitKey(1)

    def destroy_node(self):
        """Release GUI resources before node destruction."""
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    """Run the ROS image based trash detection node."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrashDetectionNode()
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
