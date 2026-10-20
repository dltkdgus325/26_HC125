#!/usr/bin/env python3
"""Serve boat diagnostics, recording controls, and map visualizations."""

from collections import deque
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64MultiArray, String
from std_srvs.srv import Trigger


TELEMETRY_LENGTH = 7
EARTH_RADIUS_M = 6378137.0


class DashboardHandler(BaseHTTPRequestHandler):
    """Handle dashboard, JSON state, and MJPEG requests."""

    dashboard = None

    def do_GET(self):
        """Serve one HTTP GET request."""
        path = urlparse(self.path).path
        if path == '/':
            self._send_file('text/html; charset=utf-8')
        elif path == '/api/state':
            self._send_json()
        elif path == '/api/heatmap':
            self._send_heatmap()
        elif path == '/stream.mjpg':
            self._send_stream()
        elif path.startswith('/maps/'):
            self._send_map(path)
        elif path == '/health':
            self._send_bytes(b'ok\n', 'text/plain; charset=utf-8')
        else:
            self.send_error(404)

    def do_POST(self):
        """Handle recording actions without exposing motion controls."""
        path = urlparse(self.path).path
        prefix = '/api/recording/'
        if not path.startswith(prefix):
            self.send_error(404)
            return
        action = path[len(prefix):]
        result = self.dashboard.trigger_recording(action)
        data = json.dumps(result, separators=(',', ':')).encode('utf-8')
        self._send_bytes(data, 'application/json; charset=utf-8')

    def log_message(self, _format, *args):
        """Suppress the default per-request stderr log."""
        del args

    def _send_file(self, content_type):
        try:
            data = self.dashboard.html_path.read_bytes()
        except OSError as exc:
            self.send_error(500, str(exc))
            return
        self._send_bytes(data, content_type)

    def _send_bytes(self, data, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self):
        data = json.dumps(
            self.dashboard.state_snapshot(),
            allow_nan=False,
            separators=(',', ':'),
        ).encode('utf-8')
        self._send_bytes(data, 'application/json; charset=utf-8')

    def _send_heatmap(self):
        data = json.dumps(
            self.dashboard.heatmap_snapshot(),
            allow_nan=False,
            separators=(',', ':'),
        ).encode('utf-8')
        self._send_bytes(data, 'application/json; charset=utf-8')

    def _send_map(self, request_path):
        filename = Path(request_path).name
        if not filename.lower().endswith('.png'):
            self.send_error(404)
            return
        try:
            data = (self.dashboard.maps_directory / filename).read_bytes()
        except OSError:
            self.send_error(404)
            return
        self._send_bytes(data, 'image/png')

    def _send_stream(self):
        self.dashboard.add_stream_client()
        self.send_response(200)
        self.send_header(
            'Content-Type',
            'multipart/x-mixed-replace; boundary=frame',
        )
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        sequence = -1
        try:
            while rclpy.ok():
                frame, sequence = self.dashboard.wait_for_frame(sequence)
                if frame is None:
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(
                    f'Content-Length: {len(frame)}\r\n\r\n'.encode('ascii')
                )
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.dashboard.remove_stream_client()


class AutonomyDashboard(Node):
    """Collect ROS diagnostics and expose them through a web interface."""

    def __init__(self):
        """Create read-only subscriptions and start the HTTP server."""
        super().__init__('autonomy_dashboard')
        defaults = {
            'host': '0.0.0.0', 'port': 8080,
            'jpeg_quality': 70, 'max_stream_fps': 10.0,
            'trash_control_config_topic': '/autonomy/trash_control_config',
            'heatmap_sample_interval': 1.0,
            'heatmap_max_samples': 600,
            'heatmap_merge_radius_m': 1.0,
            'heatmap_connect_distance_m': 3.0,
            'heatmap_ph_min': 6.0, 'heatmap_ph_max': 9.0,
            'heatmap_do_min': 0.0, 'heatmap_do_max': 12.0,
            'heatmap_turbidity_min': 0.0,
            'heatmap_turbidity_max': 100.0,
            'telemetry_topic': '/boat/telemetry',
            'telemetry_age_topic': '/boat/telemetry_age',
            'target_topic': '/trash/target',
            'debug_image_topic': '/trash/debug_image',
            'mode_topic': '/autonomy/mode',
            'global_command_topic': '/autonomy/global_cmd',
            'final_command_topic': '/boat/cmd',
            'cross_track_error_topic': '/autonomy/cross_track_error',
            'target_speed_topic': '/autonomy/target_speed',
            'raw_capture_service': '/camera/capture',
            'raw_start_service': '/camera/start_recording',
            'raw_stop_service': '/camera/stop_recording',
            'debug_start_service': '/recording/debug/start',
            'debug_stop_service': '/recording/debug/stop',
            'log_start_service': '/recording/log/start',
            'log_stop_service': '/recording/log/stop',
            'path_file': (
                'package://boat_autonomy/config/'
                'main_fountain_waypoints.yaml'
            ),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.host = str(get('host'))
        self.port = int(get('port'))
        self.jpeg_quality = int(get('jpeg_quality'))
        self.max_stream_fps = float(get('max_stream_fps'))
        self.heatmap_sample_interval = float(get('heatmap_sample_interval'))
        self.heatmap_max_samples = int(get('heatmap_max_samples'))
        self.heatmap_merge_radius_m = float(
            get('heatmap_merge_radius_m')
        )
        self.heatmap_connect_distance_m = float(
            get('heatmap_connect_distance_m')
        )
        self.heatmap_ranges = {
            'ph': [float(get('heatmap_ph_min')), float(get('heatmap_ph_max'))],
            'do': [float(get('heatmap_do_min')), float(get('heatmap_do_max'))],
            'turbidity': [
                float(get('heatmap_turbidity_min')),
                float(get('heatmap_turbidity_max')),
            ],
        }
        if not 1 <= self.port <= 65535:
            raise ValueError('port must be in [1, 65535]')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')
        if self.max_stream_fps <= 0.0:
            raise ValueError('max_stream_fps must be greater than 0')
        if self.heatmap_sample_interval <= 0.0:
            raise ValueError('heatmap_sample_interval must be > 0')
        if self.heatmap_max_samples < 1:
            raise ValueError('heatmap_max_samples must be >= 1')
        if self.heatmap_merge_radius_m <= 0.0:
            raise ValueError('heatmap_merge_radius_m must be > 0')
        if self.heatmap_connect_distance_m <= 0.0:
            raise ValueError('heatmap_connect_distance_m must be > 0')
        if any(bounds[0] >= bounds[1]
               for bounds in self.heatmap_ranges.values()):
            raise ValueError('heatmap ranges must have min < max')

        share = Path(get_package_share_directory('boat_autonomy'))
        self.html_path = share / 'web' / 'dashboard.html'
        self.maps_directory = share / 'web' / 'maps'
        self.route = self._load_route(str(get('path_file')))
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()
        self.frame_condition = threading.Condition()
        self.stream_client_count = 0
        self.jpeg_frame = None
        self.frame_sequence = 0
        self.last_encoded_at = 0.0
        self.started_at = time.monotonic()
        self.heatmap_lock = threading.Lock()
        self.heatmap_samples = deque(maxlen=self.heatmap_max_samples)
        self.last_heatmap_sample = 0.0
        self.state = {
            'mode': 'UNKNOWN', 'telemetry': None, 'telemetry_age': None,
            'target': None,
            'global_command': None, 'final_command': None,
            'cross_track_error': None, 'target_speed': None,
            'recording': {
                'raw': False, 'debug': False, 'log': False,
                'message': 'recorders ready',
            },
            'timestamps': {}, 'route': self.route,
            'display': {
                'deadband_min': None,
                'deadband_max': None,
                'deadband_bbox_gain': None,
                'bottom_deadband_start_ratio': None,
                'bottom_deadband_max': None,
            },
        }

        self.dashboard_subscriptions = [
            self.create_subscription(
                Float64MultiArray, str(get('telemetry_topic')),
                self.telemetry_callback, 10
            ),
            self.create_subscription(
                Float32, str(get('telemetry_age_topic')),
                self.telemetry_age_callback, 10
            ),
            self.create_subscription(
                Float64MultiArray, str(get('target_topic')),
                self.target_callback, 10
            ),
            self.create_subscription(
                Image, str(get('debug_image_topic')),
                self.image_callback, 1
            ),
            self.create_subscription(
                String, str(get('mode_topic')), self.mode_callback, 10
            ),
            self.create_subscription(
                Float64MultiArray, str(get('global_command_topic')),
                self.global_command_callback, 10
            ),
            self.create_subscription(
                Float64MultiArray, str(get('final_command_topic')),
                self.final_command_callback, 10
            ),
            self.create_subscription(
                Float32, str(get('cross_track_error_topic')),
                self.cross_track_callback, 10
            ),
            self.create_subscription(
                Float32, str(get('target_speed_topic')),
                self.target_speed_callback, 10
            ),
        ]
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dashboard_subscriptions.append(
            self.create_subscription(
                Float64MultiArray, str(get('trash_control_config_topic')),
                self.deadband_config_callback, config_qos
            )
        )
        self.recording_lock = threading.Lock()
        self.recording_clients = {
            action: self.create_client(Trigger, str(get(parameter)))
            for action, parameter in {
                'raw_capture': 'raw_capture_service',
                'raw_start': 'raw_start_service',
                'raw_stop': 'raw_stop_service',
                'debug_start': 'debug_start_service',
                'debug_stop': 'debug_stop_service',
                'log_start': 'log_start_service',
                'log_stop': 'log_stop_service',
            }.items()
        }

        DashboardHandler.dashboard = self
        self.http_server = ThreadingHTTPServer(
            (self.host, self.port), DashboardHandler
        )
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name='autonomy-dashboard-http',
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f'Dashboard ready: http://{self.host}:{self.port}'
        )

    def _resolve_path(self, path_file):
        prefix = 'package://'
        if path_file.startswith(prefix):
            remainder = path_file[len(prefix):]
            package, separator, relative = remainder.partition('/')
            if not package or not separator:
                raise ValueError('invalid package path')
            return Path(get_package_share_directory(package)) / relative
        return Path(path_file).expanduser().resolve()

    def _load_route(self, path_file):
        path = self._resolve_path(path_file)
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8'))
            points = data.get('waypoints', [])
            route = [
                {
                    'latitude': float(point['latitude']),
                    'longitude': float(point['longitude']),
                }
                for point in points
            ]
            background = str(data.get('map_background', '')).strip()
            raw_bounds = data.get('map_bounds')
            bounds = None
            if background and isinstance(raw_bounds, dict):
                bounds = {
                    key: float(raw_bounds[key])
                    for key in ('south', 'west', 'north', 'east')
                }
            return {
                'name': str(data.get('name', path.stem)),
                'closed_loop': bool(data.get('closed_loop', False)),
                'waypoints': route,
                'background_url': f'/maps/{Path(background).name}'
                if background else None,
                'map_bounds': bounds,
            }
        except (OSError, TypeError, KeyError, ValueError) as exc:
            self.get_logger().error(f'Cannot load dashboard route: {exc}')
            return {
                'name': path.stem, 'closed_loop': False,
                'waypoints': [], 'background_url': None,
                'map_bounds': None,
            }

    def _update(self, key, value):
        with self.state_lock:
            self.state[key] = value
            self.state['timestamps'][key] = time.monotonic()

    def telemetry_callback(self, msg):
        """Store valid STM telemetry for dashboard display."""
        if len(msg.data) < TELEMETRY_LENGTH:
            return
        values = [float(value) for value in msg.data[:TELEMETRY_LENGTH]]
        if not all(math.isfinite(value) for value in values):
            return
        telemetry = {
            'latitude': values[0], 'longitude': values[1],
            'turbidity_ntu': values[2], 'ph': values[3],
            'heading_deg': values[4], 'do_mgl': values[5],
            'gps_fix': values[6],
        }
        self._update('telemetry', telemetry)
        now = time.monotonic()
        if (values[6] > 0.0
                and now - self.last_heatmap_sample
                >= self.heatmap_sample_interval):
            sample = {
                'latitude': values[0], 'longitude': values[1],
                'turbidity_ntu': values[2], 'ph': values[3],
                'do_mgl': values[5], 'sample_count': 1,
            }
            with self.heatmap_lock:
                existing = self._nearest_heatmap_sample(sample)
                if existing is None:
                    self.heatmap_samples.append(sample)
                else:
                    existing['ph'] = sample['ph']
                    existing['do_mgl'] = sample['do_mgl']
                    existing['turbidity_ntu'] = sample['turbidity_ntu']
                    existing['sample_count'] += 1
                self.last_heatmap_sample = now

    def target_callback(self, msg):
        """Store the latest valid trash detection array."""
        if len(msg.data) < 10:
            return
        values = [float(value) for value in msg.data[:10]]
        if not all(math.isfinite(value) for value in values):
            return
        image_area = values[7] * values[8]
        area_ratio = (
            values[5] * values[6] / image_area if image_area > 0.0 else 0.0
        )
        self._update('target', {
            'detected': values[0] >= 0.5, 'class_id': values[1],
            'confidence': values[2], 'center_x': values[3],
            'center_y': values[4], 'bbox_width': values[5],
            'bbox_height': values[6], 'image_width': values[7],
            'image_height': values[8], 'track_id': values[9],
            'area_ratio': area_ratio,
        })

    def telemetry_age_callback(self, msg):
        """Store the actual UART telemetry age reported by stm_bridge."""
        value = float(msg.data)
        if math.isfinite(value) and value >= 0.0:
            self._update('telemetry_age', value)

    def image_callback(self, msg):
        """Encode selected debug frames as JPEG for MJPEG clients."""
        now = time.monotonic()
        with self.frame_condition:
            if self.stream_client_count == 0:
                return
            if now - self.last_encoded_at < 1.0 / self.max_stream_fps:
                return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Dashboard image conversion failed: {exc}'
            )
            return
        if not success:
            return
        self.last_encoded_at = now
        with self.frame_condition:
            self.jpeg_frame = encoded.tobytes()
            self.frame_sequence += 1
            self.frame_condition.notify_all()

    def add_stream_client(self):
        """Register an active MJPEG viewer."""
        with self.frame_condition:
            self.stream_client_count += 1

    def remove_stream_client(self):
        """Unregister an MJPEG viewer and release its cached frame."""
        with self.frame_condition:
            self.stream_client_count = max(0, self.stream_client_count - 1)
            if self.stream_client_count == 0:
                self.jpeg_frame = None

    def mode_callback(self, msg):
        """Store the active autonomy mode."""
        self._update('mode', str(msg.data))

    def global_command_callback(self, msg):
        """Store the latest LOS command."""
        self._store_command('global_command', msg)

    def final_command_callback(self, msg):
        """Store the latest command sent toward STM32."""
        self._store_command('final_command', msg)

    def _store_command(self, key, msg):
        if len(msg.data) < 2:
            return
        direction, output = map(float, msg.data[:2])
        if all(math.isfinite(value) for value in (direction, output)):
            self._update(key, {'direction': direction, 'output': output})

    def cross_track_callback(self, msg):
        """Store the LOS cross-track error."""
        value = float(msg.data)
        if math.isfinite(value):
            self._update('cross_track_error', value)

    def target_speed_callback(self, msg):
        """Store the LOS target speed."""
        value = float(msg.data)
        if math.isfinite(value):
            self._update('target_speed', value)

    def state_snapshot(self):
        """Return a JSON-safe copy of current diagnostics with data ages."""
        now = time.monotonic()
        with self.state_lock:
            snapshot = json.loads(json.dumps(self.state))
            timestamps = dict(self.state['timestamps'])
        snapshot['ages'] = {
            key: round(now - stamp, 3) for key, stamp in timestamps.items()
        }
        snapshot['uptime'] = round(now - self.started_at, 1)
        return snapshot

    def heatmap_snapshot(self):
        """Return a bounded copy of water-quality samples for map display."""
        with self.heatmap_lock:
            samples = [dict(sample) for sample in self.heatmap_samples]
        return {
            'samples': samples,
            'ranges': self.heatmap_ranges,
            'connect_distance_m': self.heatmap_connect_distance_m,
        }

    def _nearest_heatmap_sample(self, sample):
        """Return the nearest existing cell within the merge radius."""
        nearest = None
        nearest_distance = self.heatmap_merge_radius_m
        latitude = math.radians(sample['latitude'])
        for existing in self.heatmap_samples:
            mean_latitude = 0.5 * (
                latitude + math.radians(existing['latitude'])
            )
            east = (
                EARTH_RADIUS_M * math.cos(mean_latitude)
                * math.radians(
                    sample['longitude'] - existing['longitude']
                )
            )
            north = EARTH_RADIUS_M * math.radians(
                sample['latitude'] - existing['latitude']
            )
            distance = math.hypot(east, north)
            if distance <= nearest_distance:
                nearest = existing
                nearest_distance = distance
        return nearest

    def trigger_recording(self, action):
        """Call one allow-listed recording service from an HTTP thread."""
        client = self.recording_clients.get(action)
        if client is None:
            return {'success': False, 'message': 'unknown recording action'}
        with self.recording_lock:
            if not client.wait_for_service(timeout_sec=0.25):
                return {'success': False, 'message': 'recorder unavailable'}
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                return {'success': False, 'message': 'recorder timeout'}
            try:
                response = future.result()
            except Exception as exc:
                return {'success': False, 'message': str(exc)}
            result = {
                'success': bool(response.success),
                'message': str(response.message),
            }
            if response.success:
                state_key = {
                    'raw_start': ('raw', True),
                    'raw_stop': ('raw', False),
                    'debug_start': ('debug', True),
                    'debug_stop': ('debug', False),
                    'log_start': ('log', True),
                    'log_stop': ('log', False),
                }.get(action)
                with self.state_lock:
                    if state_key is not None:
                        self.state['recording'][state_key[0]] = state_key[1]
                    self.state['recording']['message'] = response.message
            return result

    def deadband_config_callback(self, message):
        """Store deadband values published by the active controller."""
        if len(message.data) < 3:
            return
        minimum, maximum, gain = map(float, message.data[:3])
        if not (0.0 <= minimum <= maximum < 1.0 and gain >= 0.0):
            return
        with self.state_lock:
            self.state['display'] = {
                'deadband_min': minimum,
                'deadband_max': maximum,
                'deadband_bbox_gain': gain,
                'bottom_deadband_start_ratio': (
                    float(message.data[3]) if len(message.data) >= 5 else None
                ),
                'bottom_deadband_max': (
                    float(message.data[4]) if len(message.data) >= 5 else None
                ),
            }

    def wait_for_frame(self, sequence):
        """Wait briefly for a newer JPEG frame and return it."""
        with self.frame_condition:
            self.frame_condition.wait_for(
                lambda: self.frame_sequence != sequence or not rclpy.ok(),
                timeout=1.0,
            )
            return self.jpeg_frame, self.frame_sequence

    def destroy_node(self):
        """Stop the HTTP server before node destruction."""
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=1.0)
        with self.frame_condition:
            self.frame_condition.notify_all()
        return super().destroy_node()


def main(args=None):
    """Run the read-only autonomy dashboard node."""
    rclpy.init(args=args)
    node = None
    try:
        node = AutonomyDashboard()
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
