#!/usr/bin/env python3
"""Record annotated video and combined telemetry/command logs on demand."""

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64MultiArray, String
from std_srvs.srv import Trigger


LOG_HEADER = [
    'pc_time', 'latitude', 'longitude', 'turbidity_ntu', 'ph',
    'heading_deg', 'do_mgl', 'gps_fix', 'tel_age',
    'mode', 'target_state', 'target_detected', 'target_class_id',
    'target_confidence', 'target_center_x', 'target_center_y',
    'target_bbox_width', 'target_bbox_height', 'target_image_width',
    'target_image_height', 'target_track_id',
    'global_direction_deg', 'global_output',
    'direction_deg', 'output', 'cmd_age', 'system_event',
]


class MissionRecorder(Node):
    """Provide isolated debug-video and combined-log recording services."""

    def __init__(self):
        """Create read-only subscriptions and on-demand recorders."""
        super().__init__('mission_recorder')
        defaults = {
            'debug_image_topic': '/trash/debug_image',
            'telemetry_topic': '/boat/telemetry',
            'telemetry_age_topic': '/boat/telemetry_age',
            'command_topic': '/boat/cmd',
            'global_command_topic': '/autonomy/global_cmd',
            'mode_topic': '/autonomy/mode',
            'target_topic': '/trash/target',
            'target_state_topic': '/trash/target_state',
            'system_event_topic': '/autonomy/system_event',
            'output_directory': '/home/rodin/boat_data/mission_records',
            'debug_video_fps': 10.0, 'debug_video_codec': 'mp4v',
            'log_interval': 0.10, 'log_flush_interval': 1.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def get(name):
            return self.get_parameter(name).value

        self.debug_video_fps = float(get('debug_video_fps'))
        self.debug_video_codec = str(get('debug_video_codec'))
        self.log_interval = float(get('log_interval'))
        self.log_flush_interval = float(get('log_flush_interval'))
        if self.debug_video_fps <= 0.0 or self.log_interval <= 0.0:
            raise ValueError('recording rates must be > 0')
        if self.log_flush_interval <= 0.0:
            raise ValueError('log_flush_interval must be > 0')
        if len(self.debug_video_codec) != 4:
            raise ValueError('debug_video_codec must be four characters')

        self.output_directory = Path(
            str(get('output_directory'))
        ).expanduser()
        self.video_directory = self.output_directory / 'debug_videos'
        self.log_directory = self.output_directory / 'logs'
        self.video_directory.mkdir(parents=True, exist_ok=True)
        self.log_directory.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.data_lock = threading.Lock()
        self.latest_debug_frame = None
        self.debug_frame_sequence = 0
        self.written_debug_sequence = 0
        self.last_debug_capture = 0.0
        self.latest_telemetry = None
        self.latest_telemetry_age = None
        self.latest_command = None
        self.latest_command_time = None
        self.latest_global_command = None
        self.latest_mode = ''
        self.latest_target_state = ''
        self.latest_target = None
        self.latest_system_event = ''
        self.video_requested = False
        self.video_writer = None
        self.video_path = None
        self.log_file = None
        self.log_writer = None
        self.log_path = None
        self.last_log_flush = time.monotonic()

        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._recording_subscriptions = [
            self.create_subscription(
                Image, str(get('debug_image_topic')),
                self.image_callback, qos_profile_sensor_data
            ),
            self.create_subscription(
                Float64MultiArray, str(get('telemetry_topic')),
                self.telemetry_callback, 10
            ),
            self.create_subscription(
                Float32, str(get('telemetry_age_topic')),
                self.telemetry_age_callback, 10
            ),
            self.create_subscription(
                Float64MultiArray, str(get('command_topic')),
                self.command_callback, 10
            ),
            self.create_subscription(
                Float64MultiArray, str(get('global_command_topic')),
                self.global_command_callback, 10
            ),
            self.create_subscription(
                String, str(get('mode_topic')), self.mode_callback, 10
            ),
            self.create_subscription(
                Float64MultiArray, str(get('target_topic')),
                self.target_callback, 10
            ),
            self.create_subscription(
                String, str(get('target_state_topic')),
                self.target_state_callback, 10
            ),
            self.create_subscription(
                String, str(get('system_event_topic')),
                self.system_event_callback, event_qos
            ),
        ]
        self._recording_services = [
            self.create_service(
                Trigger, '/recording/debug/start', self.start_debug_video
            ),
            self.create_service(
                Trigger, '/recording/debug/stop', self.stop_debug_video
            ),
            self.create_service(
                Trigger, '/recording/log/start', self.start_log
            ),
            self.create_service(
                Trigger, '/recording/log/stop', self.stop_log
            ),
        ]
        self.video_timer = self.create_timer(
            1.0 / self.debug_video_fps, self.write_debug_frame
        )
        self.log_timer = self.create_timer(
            self.log_interval, self.write_log_row
        )
        self.get_logger().info(
            f'Mission recorder ready: {self.output_directory}'
        )

    def image_callback(self, message):
        """Convert at most the configured FPS and keep only the newest frame."""
        now = time.monotonic()
        with self.data_lock:
            if not self.video_requested:
                return
            interval = 1.0 / self.debug_video_fps
            if now - self.last_debug_capture < interval:
                return
            self.last_debug_capture = now
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().warning(f'Debug image conversion failed: {exc}')
            return
        with self.data_lock:
            if self.video_requested and self.video_writer is None:
                height, width = frame.shape[:2]
                writer = cv2.VideoWriter(
                    str(self.video_path),
                    cv2.VideoWriter_fourcc(*self.debug_video_codec),
                    self.debug_video_fps, (width, height)
                )
                if not writer.isOpened():
                    self.video_requested = False
                    self.video_path = None
                    self.get_logger().error('Cannot open debug video writer')
                    return
                self.video_writer = writer
            self.latest_debug_frame = frame
            self.debug_frame_sequence += 1

    def telemetry_callback(self, message):
        """Keep the latest complete finite telemetry sample."""
        if len(message.data) < 7:
            return
        values = tuple(map(float, message.data[:7]))
        if all(math.isfinite(value) for value in values):
            with self.data_lock:
                self.latest_telemetry = values

    def command_callback(self, message):
        """Keep the latest finite final command sample."""
        if len(message.data) < 2:
            return
        values = tuple(map(float, message.data[:2]))
        if all(math.isfinite(value) for value in values):
            with self.data_lock:
                self.latest_command = values
                self.latest_command_time = time.monotonic()

    def telemetry_age_callback(self, message):
        """Store the UART TEL packet age reported by stm_bridge."""
        value = float(message.data)
        if math.isfinite(value):
            with self.data_lock:
                self.latest_telemetry_age = (
                    value, time.monotonic()
                )

    def global_command_callback(self, message):
        """Keep the latest finite LOS command without affecting control."""
        values = self._finite_values(message, 2)
        if values is not None:
            with self.data_lock:
                self.latest_global_command = values

    def mode_callback(self, message):
        """Keep the latest autonomy mode."""
        with self.data_lock:
            self.latest_mode = str(message.data)

    def target_state_callback(self, message):
        """Keep the temporal filter state emitted by trash detection."""
        with self.data_lock:
            self.latest_target_state = str(message.data)

    def target_callback(self, message):
        """Keep the complete confirmed-target transport array."""
        values = self._finite_values(message, 10)
        if values is not None:
            with self.data_lock:
                self.latest_target = values

    @staticmethod
    def _finite_values(message, count):
        if len(message.data) < count:
            return None
        values = tuple(map(float, message.data[:count]))
        return values if all(math.isfinite(value) for value in values) else None

    def system_event_callback(self, message):
        """Keep the latest retained system warning or error."""
        with self.data_lock:
            self.latest_system_event = str(message.data)

    def start_debug_video(self, _request, response):
        """Request annotated-video recording from the next frame."""
        with self.data_lock:
            if self.video_requested:
                return self._respond(
                    response, False, f'already recording: {self.video_path}'
                )
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = self.video_directory / f'debug_{stamp}.mp4'
            self.video_requested = True
            self.video_path = path
            self.latest_debug_frame = None
            self.debug_frame_sequence = 0
            self.written_debug_sequence = 0
            self.last_debug_capture = 0.0
        self.get_logger().info(f'Debug recording started: {path}')
        return self._respond(response, True, str(path))

    def stop_debug_video(self, _request, response):
        """Stop annotated-video recording."""
        with self.data_lock:
            if not self.video_requested:
                return self._respond(response, False, 'not recording')
            path = self.video_path
            if self.video_writer is not None:
                self.video_writer.release()
            self.video_requested = False
            self.video_writer = None
            self.video_path = None
        self.get_logger().info(f'Debug recording stopped: {path}')
        return self._respond(response, True, str(path))

    def write_debug_frame(self):
        """Write at most one newest frame per configured interval."""
        with self.data_lock:
            if self.video_writer is None or self.latest_debug_frame is None:
                return
            if self.written_debug_sequence == self.debug_frame_sequence:
                return
            self.video_writer.write(self.latest_debug_frame)
            self.written_debug_sequence = self.debug_frame_sequence

    def start_log(self, _request, response):
        """Start one combined telemetry and final-command CSV."""
        with self.data_lock:
            if self.log_file is not None:
                return self._respond(
                    response, False, f'already logging: {self.log_path}'
                )
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = self.log_directory / f'mission_{stamp}.csv'
            log_file = path.open('w', newline='', encoding='utf-8')
            writer = csv.writer(log_file)
            writer.writerow(LOG_HEADER)
            log_file.flush()
            self.log_file = log_file
            self.log_writer = writer
            self.log_path = path
            self.last_log_flush = time.monotonic()
        self.get_logger().info(f'Mission log started: {path}')
        return self._respond(response, True, str(path))

    def stop_log(self, _request, response):
        """Stop and close the combined CSV log."""
        with self.data_lock:
            if self.log_file is None:
                return self._respond(response, False, 'not logging')
            path = self.log_path
            self.log_file.flush()
            self.log_file.close()
            self.log_file = None
            self.log_writer = None
            self.log_path = None
        self.get_logger().info(f'Mission log stopped: {path}')
        return self._respond(response, True, str(path))

    def write_log_row(self):
        """Append the latest telemetry and command without blocking control."""
        with self.data_lock:
            if self.log_writer is None:
                return
            telemetry = self.latest_telemetry or ('',) * 7
            target = self.latest_target or ('',) * 10
            global_command = self.latest_global_command or ('',) * 2
            command = self.latest_command or ('',) * 2
            now = time.monotonic()
            if self.latest_telemetry_age is None:
                telemetry_age = ''
            else:
                base_age, received_at = self.latest_telemetry_age
                telemetry_age = (
                    base_age + now - received_at if base_age >= 0.0 else ''
                )
            command_age = (
                now - self.latest_command_time
                if self.latest_command_time is not None else ''
            )
            self.log_writer.writerow([
                datetime.now().isoformat(timespec='milliseconds'),
                *telemetry, telemetry_age, self.latest_mode,
                self.latest_target_state, *target, *global_command,
                *command, command_age,
                self.latest_system_event,
            ])
            if now - self.last_log_flush >= self.log_flush_interval:
                self.log_file.flush()
                self.last_log_flush = now

    @staticmethod
    def _respond(response, success, message):
        response.success = success
        response.message = message
        return response

    def destroy_node(self):
        """Close active recording files."""
        with self.data_lock:
            if self.video_writer is not None:
                self.video_writer.release()
            if self.log_file is not None:
                self.log_file.flush()
                self.log_file.close()
        return super().destroy_node()


def main(args=None):
    """Run the isolated mission recording node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MissionRecorder()
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
