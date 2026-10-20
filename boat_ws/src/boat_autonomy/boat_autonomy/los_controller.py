#!/usr/bin/env python3
"""LOS waypoint controller for the ROS interface exposed by stm_bridge.py."""

import math
import statistics
import time
from collections import deque
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, String


EARTH_RADIUS_M = 6378137.0
TELEMETRY_LENGTH = 7
LATITUDE_INDEX = 0
LONGITUDE_INDEX = 1
HEADING_INDEX = 4
GPS_FIX_INDEX = 6


def clamp(value, low, high):
    """Clamp value to the inclusive range."""
    return max(low, min(high, value))


def wrap_angle(angle):
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class LOSController(Node):
    """Follow configured GPS waypoints and publish STM32-ready commands."""

    def __init__(self):
        """Initialize parameters, controller state, and ROS interfaces."""
        super().__init__('los_controller')

        self.declare_parameter('telemetry_topic', '/boat/telemetry')
        self.declare_parameter('command_topic', '/boat/cmd')
        self.declare_parameter('mode_topic', '/autonomy/mode')
        self.declare_parameter('target_speed_topic', '/autonomy/target_speed')
        self.declare_parameter(
            'cross_track_error_topic', '/autonomy/cross_track_error'
        )
        self.declare_parameter('lookahead', 3.0)
        self.declare_parameter('waypoint_radius', 2.0)
        self.declare_parameter('max_rudder_deg', 30.0)
        self.declare_parameter('heading_kp', 1.2)
        self.declare_parameter('heading_kd', 0.0)
        self.declare_parameter('heading_filter_alpha', 0.3)
        self.declare_parameter('yaw_rate_filter_alpha', 0.25)
        self.declare_parameter('max_rudder_rate_deg_s', 30.0)
        self.declare_parameter('cruise_speed', 1.0)
        self.declare_parameter('speed_kp', 5.0)
        self.declare_parameter('speed_ki', 1.0)
        self.declare_parameter('speed_filter_alpha', 0.3)
        self.declare_parameter('speed_integral_limit', 5.0)
        self.declare_parameter('turn_speed_reduction', 0.5)
        self.declare_parameter('minimum_turn_output', 1.0)
        self.declare_parameter('max_output', 10.0)
        self.declare_parameter('max_valid_gps_speed', 8.0)
        self.declare_parameter('gps_min_displacement', 0.10)
        self.declare_parameter('gps_stationary_timeout', 2.0)
        self.declare_parameter('gps_median_window', 3)
        self.declare_parameter('join_nearest_segment', True)
        self.declare_parameter('start_waypoint_index', 0)
        self.declare_parameter('max_laps', 1)
        self.declare_parameter(
            'path_file',
            'package://boat_autonomy/config/example_route.yaml',
        )

        self.lookahead = float(self.get_parameter('lookahead').value)
        self.wp_radius = float(self.get_parameter('waypoint_radius').value)
        self.max_rudder_deg = float(
            self.get_parameter('max_rudder_deg').value
        )
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.heading_filter_alpha = float(
            self.get_parameter('heading_filter_alpha').value
        )
        self.yaw_rate_filter_alpha = float(
            self.get_parameter('yaw_rate_filter_alpha').value
        )
        self.max_rudder_rate_deg_s = float(
            self.get_parameter('max_rudder_rate_deg_s').value
        )
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.speed_kp = float(self.get_parameter('speed_kp').value)
        self.speed_ki = float(self.get_parameter('speed_ki').value)
        self.speed_filter_alpha = float(
            self.get_parameter('speed_filter_alpha').value
        )
        self.speed_integral_limit = float(
            self.get_parameter('speed_integral_limit').value
        )
        self.turn_speed_reduction = float(
            self.get_parameter('turn_speed_reduction').value
        )
        self.minimum_turn_output = float(
            self.get_parameter('minimum_turn_output').value
        )
        self.max_output = float(self.get_parameter('max_output').value)
        self.max_valid_gps_speed = float(
            self.get_parameter('max_valid_gps_speed').value
        )
        self.gps_min_displacement = float(
            self.get_parameter('gps_min_displacement').value
        )
        self.gps_stationary_timeout = float(
            self.get_parameter('gps_stationary_timeout').value
        )
        self.gps_median_window = int(
            self.get_parameter('gps_median_window').value
        )
        self.join_nearest_segment = bool(
            self.get_parameter('join_nearest_segment').value
        )
        self.start_waypoint_index = int(
            self.get_parameter('start_waypoint_index').value
        )
        self.max_laps = int(self.get_parameter('max_laps').value)
        self.path_file = str(self.get_parameter('path_file').value)
        self.waypoints = self._load_route(self.path_file)
        self._validate_parameters()

        self.lat0 = None
        self.lon0 = None
        self.waypoints_xy = []
        self.gps_window = deque(maxlen=self.gps_median_window)
        self.accepted_latitude = None
        self.accepted_longitude = None
        self.accepted_gps_time = None
        self.last_gps_rejection_log = 0.0
        self.last_control_time = None
        self.heading = None
        self.last_heading_time = None
        self.yaw_rate = 0.0
        self.last_rudder_deg = 0.0
        self.speed = 0.0
        self.wp_index = 0
        self.completed_laps = 0
        self.waypoints_advanced = 0
        self.speed_integral = 0.0
        self.last_invalid_telemetry_log = 0.0
        self.route_initialized = False
        self.autonomy_mode = 'UNKNOWN'

        telemetry_topic = str(self.get_parameter('telemetry_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        self.command_pub = self.create_publisher(
            Float64MultiArray, command_topic, 10
        )
        self.target_speed_pub = self.create_publisher(
            Float32,
            self.get_parameter('target_speed_topic').value,
            10,
        )
        self.cross_track_pub = self.create_publisher(
            Float32,
            self.get_parameter('cross_track_error_topic').value,
            10,
        )
        self.telemetry_sub = self.create_subscription(
            Float64MultiArray, telemetry_topic, self.telemetry_callback, 10
        )
        self.mode_sub = self.create_subscription(
            String, str(self.get_parameter('mode_topic').value),
            self.mode_callback, 10
        )
        self.get_logger().info(
            f'LOS controller started: {telemetry_topic} -> {command_topic} '
            f'({len(self.waypoints)} waypoints, '
            f'closed_loop={self.closed_loop}, from {self.path_file})'
        )
        if not self.survey_grade:
            self.get_logger().warning(
                'Route is not marked survey-grade; verify it on site'
            )
        if self.route_warning:
            self.get_logger().warning(f'Route warning: {self.route_warning}')

    def mode_callback(self, msg):
        """Rejoin the nearest route segment after trash approach ends."""
        new_mode = str(msg.data).upper()
        returning_to_global = (
            self.autonomy_mode in ('APPROACH', 'COLLECT')
            and new_mode == 'GLOBAL'
        )
        self.autonomy_mode = new_mode
        if not returning_to_global:
            return
        self.route_initialized = False
        self.speed_integral = 0.0
        self.last_rudder_deg = 0.0
        self.publish_control(0.0, 0.0, 0.0)
        self.get_logger().info(
            'Trash mission ended: nearest LOS segment will be rejoined'
        )

    def _resolve_path_file(self, path_file):
        """Resolve an absolute, relative, home, or package URI path."""
        package_prefix = 'package://'
        if path_file.startswith(package_prefix):
            remainder = path_file[len(package_prefix):]
            package_name, separator, relative_path = remainder.partition('/')
            if not package_name or not separator or not relative_path:
                raise ValueError(
                    'path_file package URI must be '
                    'package://<package>/<relative-path>'
                )
            share_dir = Path(get_package_share_directory(package_name))
            return share_dir / relative_path
        return Path(path_file).expanduser().resolve()

    def _load_route(self, path_file):
        """Read and validate WGS84 waypoints from a route YAML file."""
        if not path_file.strip():
            raise ValueError('path_file must not be empty')
        route_path = self._resolve_path_file(path_file)
        try:
            with route_path.open('r', encoding='utf-8') as route_file:
                route_data = yaml.safe_load(route_file)
        except OSError as exc:
            raise ValueError(
                f'cannot read path_file {route_path}: {exc}'
            ) from exc
        if not isinstance(route_data, dict):
            raise ValueError('route YAML root must be a mapping')
        coordinate_system = route_data.get(
            'coordinate_system',
            route_data.get('coordinate_frame', 'wgs84'),
        )
        if str(coordinate_system).lower() != 'wgs84':
            raise ValueError('route coordinate_system must be wgs84')
        closed_loop = route_data.get('closed_loop', False)
        if not isinstance(closed_loop, bool):
            raise ValueError('route closed_loop must be true or false')
        self.closed_loop = closed_loop
        self.survey_grade = route_data.get('survey_grade', False) is True
        self.route_warning = str(route_data.get('warning', '')).strip()
        raw_waypoints = route_data.get('waypoints')
        if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
            raise ValueError('route must contain at least two waypoints')

        waypoints = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError(f'waypoint {index} must be a mapping')
            try:
                latitude = float(waypoint['latitude'])
                longitude = float(waypoint['longitude'])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f'waypoint {index} requires numeric latitude/longitude'
                ) from exc
            if not math.isfinite(latitude) or not -90.0 <= latitude <= 90.0:
                raise ValueError(
                    f'invalid waypoint {index} latitude: {latitude}'
                )
            valid_longitude = (
                math.isfinite(longitude)
                and -180.0 <= longitude <= 180.0
            )
            if not valid_longitude:
                raise ValueError(
                    f'invalid waypoint {index} longitude: {longitude}'
                )
            waypoints.append((latitude, longitude))

        # A closed route may repeat its first point as the final point. The
        # controller represents that closing edge implicitly, so remove it.
        if self.closed_loop and waypoints[-1] == waypoints[0]:
            waypoints.pop()
        minimum_points = 3 if self.closed_loop else 2
        if len(waypoints) < minimum_points:
            raise ValueError(
                f'route requires at least {minimum_points} distinct waypoints'
            )
        for index in range(len(waypoints) - 1):
            if waypoints[index] == waypoints[index + 1]:
                raise ValueError(
                    f'consecutive duplicate waypoints at {index}/{index + 1}'
                )
        return waypoints

    def _validate_parameters(self):
        positive = {
            'lookahead': self.lookahead,
            'waypoint_radius': self.wp_radius,
            'speed_integral_limit': self.speed_integral_limit,
            'max_valid_gps_speed': self.max_valid_gps_speed,
            'gps_min_displacement': self.gps_min_displacement,
            'gps_stationary_timeout': self.gps_stationary_timeout,
            'max_rudder_rate_deg_s': self.max_rudder_rate_deg_s,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and > 0')
        if not 0.0 < self.max_rudder_deg <= 180.0:
            raise ValueError('max_rudder_deg must be in (0, 180]')
        if not 0.0 < self.max_output <= 10.0:
            raise ValueError('max_output must be in (0, 10]')
        if not 0.0 <= self.minimum_turn_output <= self.max_output:
            raise ValueError(
                'minimum_turn_output must be in [0, max_output]'
            )
        if self.cruise_speed < 0.0:
            raise ValueError('cruise_speed must be >= 0')
        if not 0.0 < self.heading_filter_alpha <= 1.0:
            raise ValueError('heading_filter_alpha must be in (0, 1]')
        if self.heading_kd < 0.0:
            raise ValueError('heading_kd must be >= 0')
        if not 0.0 < self.yaw_rate_filter_alpha <= 1.0:
            raise ValueError('yaw_rate_filter_alpha must be in (0, 1]')
        if not 0.0 < self.speed_filter_alpha <= 1.0:
            raise ValueError('speed_filter_alpha must be in (0, 1]')
        if not 0.0 <= self.turn_speed_reduction <= 1.0:
            raise ValueError('turn_speed_reduction must be in [0, 1]')
        if self.max_laps < 0:
            raise ValueError('max_laps must be >= 0')
        if self.gps_median_window < 3 or self.gps_median_window % 2 == 0:
            raise ValueError('gps_median_window must be an odd integer >= 3')
        final_start_index = (
            len(self.waypoints) - 1 if self.closed_loop
            else len(self.waypoints) - 2
        )
        if not 0 <= self.start_waypoint_index <= final_start_index:
            raise ValueError(
                'start_waypoint_index does not identify a route segment'
            )

    def gps_to_xy(self, latitude, longitude):
        """Convert WGS84 coordinates to local tangent-plane metres."""
        x = (
            EARTH_RADIUS_M
            * math.cos(math.radians(self.lat0))
            * math.radians(longitude - self.lon0)
        )
        y = EARTH_RADIUS_M * math.radians(latitude - self.lat0)
        return x, y

    def telemetry_callback(self, msg):
        """Consume stm_bridge telemetry and publish a fresh command."""
        if len(msg.data) < TELEMETRY_LENGTH:
            self._invalid_telemetry(
                f'expected {TELEMETRY_LENGTH} values, got {len(msg.data)}'
            )
            return
        heading_deg = float(msg.data[HEADING_INDEX])
        latitude = float(msg.data[LATITUDE_INDEX])
        longitude = float(msg.data[LONGITUDE_INDEX])
        gps_fix = float(msg.data[GPS_FIX_INDEX])
        if not all(math.isfinite(value) for value in (
            heading_deg, latitude, longitude, gps_fix
        )):
            self._invalid_telemetry('non-finite heading/GPS value')
            return
        if gps_fix <= 0.0 or not (-90.0 <= latitude <= 90.0) or not (
            -180.0 <= longitude <= 180.0
        ):
            self._invalid_telemetry('GPS fix or coordinate is invalid')
            return

        now = time.monotonic()

        # STM heading: 0=N and clockwise-positive. Internal ENU: 0=E, CCW.
        raw_heading = wrap_angle(math.radians(90.0 - heading_deg))
        if self.heading is None:
            self.heading = raw_heading
            self.last_heading_time = now
        else:
            heading_delta = wrap_angle(raw_heading - self.heading)
            filtered_heading = wrap_angle(
                self.heading + self.heading_filter_alpha * heading_delta
            )
            heading_dt = now - self.last_heading_time
            if heading_dt > 0.001:
                measured_yaw_rate = (
                    wrap_angle(filtered_heading - self.heading) / heading_dt
                )
                self.yaw_rate = (
                    self.yaw_rate_filter_alpha * measured_yaw_rate
                    + (1.0 - self.yaw_rate_filter_alpha) * self.yaw_rate
                )
            self.heading = filtered_heading
            self.last_heading_time = now

        position = self._filter_gps(latitude, longitude, now)
        if position is None:
            if self.accepted_latitude is None:
                self.publish_control(0.0, 0.0, 0.0)
                return
            latitude = self.accepted_latitude
            longitude = self.accepted_longitude
        else:
            latitude, longitude = position

        if self.lat0 is None:
            self.lat0 = latitude
            self.lon0 = longitude
            self.waypoints_xy = [
                self.gps_to_xy(lat, lon)
                for lat, lon in self.waypoints
            ]
            self.get_logger().info(f'GPS origin: {self.lat0}, {self.lon0}')

        x, y = self.gps_to_xy(latitude, longitude)
        if not self.route_initialized:
            if self.join_nearest_segment:
                self.wp_index, route_distance = self._nearest_segment(x, y)
                join_description = (
                    f'nearest segment {self.wp_index} -> '
                    f'{self.wp_index + 1} ({route_distance:.2f} m away)'
                )
            else:
                self.wp_index = self.start_waypoint_index
                join_description = (
                    f'configured segment {self.wp_index} -> '
                    f'{self.wp_index + 1}'
                )
            self.route_initialized = True
            self.get_logger().info(f'Route joined at {join_description}')
        controller_dt = (
            None
            if self.last_control_time is None
            else now - self.last_control_time
        )
        self.last_control_time = now
        if (
            self.accepted_gps_time is not None
            and now - self.accepted_gps_time >= self.gps_stationary_timeout
        ):
            self.speed = 0.0
        self.run_controller(x, y, controller_dt)

    def _filter_gps(self, latitude, longitude, now):
        """Median-filter GPS and reject implausible position jumps."""
        self.gps_window.append((latitude, longitude))
        candidate_latitude = statistics.median(
            point[0] for point in self.gps_window
        )
        candidate_longitude = statistics.median(
            point[1] for point in self.gps_window
        )
        if self.accepted_latitude is None:
            self.accepted_latitude = candidate_latitude
            self.accepted_longitude = candidate_longitude
            self.accepted_gps_time = now
            return candidate_latitude, candidate_longitude

        old_x, old_y = self._relative_gps_delta(
            self.accepted_latitude,
            self.accepted_longitude,
            candidate_latitude,
            candidate_longitude,
        )
        distance = math.hypot(old_x, old_y)
        if distance < self.gps_min_displacement:
            return self.accepted_latitude, self.accepted_longitude

        sample_dt = now - self.accepted_gps_time
        if sample_dt <= 0.01:
            return None
        candidate_speed = distance / sample_dt
        if candidate_speed > self.max_valid_gps_speed:
            if now - self.last_gps_rejection_log >= 1.0:
                self.get_logger().warning(
                    'GPS position jump rejected: '
                    f'{distance:.2f} m / {sample_dt:.2f} s '
                    f'= {candidate_speed:.2f} m/s'
                )
                self.last_gps_rejection_log = now
            return None

        self.speed = (
            self.speed_filter_alpha * candidate_speed
            + (1.0 - self.speed_filter_alpha) * self.speed
        )
        self.accepted_latitude = candidate_latitude
        self.accepted_longitude = candidate_longitude
        self.accepted_gps_time = now
        return candidate_latitude, candidate_longitude

    @staticmethod
    def _relative_gps_delta(lat1, lon1, lat2, lon2):
        """Return nearby WGS84 displacement as east/north metres."""
        mean_latitude = math.radians((lat1 + lat2) * 0.5)
        east = (
            EARTH_RADIUS_M
            * math.cos(mean_latitude)
            * math.radians(lon2 - lon1)
        )
        north = EARTH_RADIUS_M * math.radians(lat2 - lat1)
        return east, north

    def _nearest_segment(self, x, y):
        """Return the closest route segment index and projection distance."""
        best_index = 0
        best_distance = math.inf
        segment_count = (
            len(self.waypoints_xy)
            if self.closed_loop
            else len(self.waypoints_xy) - 1
        )
        for index in range(segment_count):
            x1, y1 = self.waypoints_xy[index]
            x2, y2 = self.waypoints_xy[
                (index + 1) % len(self.waypoints_xy)
            ]
            segment_dx = x2 - x1
            segment_dy = y2 - y1
            length_squared = segment_dx ** 2 + segment_dy ** 2
            if length_squared <= 1e-12:
                projection_x, projection_y = x1, y1
            else:
                projection = clamp(
                    ((x - x1) * segment_dx + (y - y1) * segment_dy)
                    / length_squared,
                    0.0,
                    1.0,
                )
                projection_x = x1 + projection * segment_dx
                projection_y = y1 + projection * segment_dy
            distance = math.hypot(x - projection_x, y - projection_y)
            if distance < best_distance:
                best_index = index
                best_distance = distance
        return best_index, best_distance

    def _invalid_telemetry(self, reason):
        now = time.monotonic()
        if now - self.last_invalid_telemetry_log >= 1.0:
            self.get_logger().warning(f'Invalid telemetry -> STOP: {reason}')
            self.last_invalid_telemetry_log = now
        self.speed_integral = 0.0
        self.publish_control(0.0, 0.0, 0.0)

    def run_controller(self, x, y, controller_dt):
        """Run LOS guidance, heading control, and speed PI control."""
        waypoint_count = len(self.waypoints_xy)
        # Limit advancement to one full route per callback. This avoids an
        # infinite loop when an invalid tiny closed route fits in the radius.
        for _ in range(waypoint_count):
            if not self.closed_loop and self.wp_index >= waypoint_count - 1:
                break
            next_index = (self.wp_index + 1) % waypoint_count
            x1, y1 = self.waypoints_xy[self.wp_index]
            x2, y2 = self.waypoints_xy[next_index]
            segment_dx = x2 - x1
            segment_dy = y2 - y1
            length_squared = segment_dx ** 2 + segment_dy ** 2
            if length_squared <= 1e-12:
                self.get_logger().error(
                    'Consecutive route points collapse locally -> STOP'
                )
                self.publish_control(0.0, 0.0, 0.0)
                return
            progress = (
                (x - x1) * segment_dx + (y - y1) * segment_dy
            ) / length_squared
            reached = math.hypot(x2 - x, y2 - y) < self.wp_radius
            passed = progress >= 1.0
            if not reached and not passed:
                break
            self.wp_index = next_index
            self.waypoints_advanced += 1
            self.speed_integral = 0.0
            self.get_logger().info(f'Waypoint reached -> {self.wp_index}')
            new_lap_count = self.waypoints_advanced // waypoint_count
            if self.closed_loop and new_lap_count > self.completed_laps:
                self.completed_laps = new_lap_count
                self.get_logger().info(
                    f'Closed-route lap completed: {self.completed_laps}'
                )
                if self.max_laps and self.completed_laps >= self.max_laps:
                    self.publish_control(0.0, 0.0, 0.0)
                    return

        if not self.closed_loop and self.wp_index >= waypoint_count - 1:
            self.publish_control(0.0, 0.0, 0.0)
            return

        x1, y1 = self.waypoints_xy[self.wp_index]
        x2, y2 = self.waypoints_xy[
            (self.wp_index + 1) % waypoint_count
        ]
        segment_dx = x2 - x1
        segment_dy = y2 - y1
        if math.hypot(segment_dx, segment_dy) < 1e-6:
            self.get_logger().error(
                'Consecutive waypoints are identical -> STOP'
            )
            self.publish_control(0.0, 0.0, 0.0)
            return

        path_heading = math.atan2(segment_dy, segment_dx)
        dx = x - x1
        dy = y - y1
        cross_track_error = (
            -dx * math.sin(path_heading) + dy * math.cos(path_heading)
        )
        desired_heading = wrap_angle(
            path_heading - math.atan2(cross_track_error, self.lookahead)
        )
        heading_error = wrap_angle(desired_heading - self.heading)
        # STM direction: signed right/clockwise steering command in degrees.
        rudder_target = clamp(
            -self.heading_kp * math.degrees(heading_error)
            + self.heading_kd * math.degrees(self.yaw_rate),
            -self.max_rudder_deg,
            self.max_rudder_deg,
        )
        rudder_dt = (
            controller_dt
            if controller_dt is not None and 0.0 < controller_dt <= 1.0
            else 0.05
        )
        max_rudder_change = self.max_rudder_rate_deg_s * rudder_dt
        rudder_deg = clamp(
            rudder_target,
            self.last_rudder_deg - max_rudder_change,
            self.last_rudder_deg + max_rudder_change,
        )
        self.last_rudder_deg = rudder_deg
        heading_ratio = min(abs(heading_error) / math.radians(90.0), 1.0)
        target_speed = self.cruise_speed * (
            1.0 - self.turn_speed_reduction * heading_ratio
        )
        speed_error = target_speed - self.speed
        if controller_dt is not None and 0.0 < controller_dt <= 1.0:
            self.speed_integral = clamp(
                self.speed_integral + speed_error * controller_dt,
                -self.speed_integral_limit,
                self.speed_integral_limit,
            )
        throttle = clamp(
            self.speed_kp * speed_error + self.speed_ki * self.speed_integral,
            0.0,
            self.max_output,
        )
        # The STM32 cannot steer without propulsion. Apply only the minimum
        # output needed for the requested turn, proportional to rudder angle.
        # Explicit STOP paths bypass this controller block and remain (0, 0).
        steering_ratio = min(
            abs(rudder_deg) / self.max_rudder_deg,
            1.0,
        )
        throttle = max(
            throttle,
            self.minimum_turn_output * steering_ratio,
        )
        self.publish_control(rudder_deg, throttle, target_speed)

        cte_msg = Float32()
        cte_msg.data = float(cross_track_error)
        self.cross_track_pub.publish(cte_msg)

    def publish_control(self, rudder_deg, throttle, target_speed):
        """Publish an STM-ready command and controller diagnostics."""
        if rudder_deg == 0.0 and throttle == 0.0:
            self.last_rudder_deg = 0.0
        command_msg = Float64MultiArray()
        command_msg.data = [float(rudder_deg), float(throttle)]
        self.command_pub.publish(command_msg)
        speed_msg = Float32()
        speed_msg.data = float(target_speed)
        self.target_speed_pub.publish(speed_msg)

    def destroy_node(self):
        """Publish a final stop command before destroying the ROS node."""
        if rclpy.ok():
            try:
                self.publish_control(0.0, 0.0, 0.0)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    """Run the LOS controller node."""
    rclpy.init(args=args)
    node = None
    try:
        node = LOSController()
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
