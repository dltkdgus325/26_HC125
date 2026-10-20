#!/usr/bin/env python3
"""Periodically save boat telemetry and GPS coordinates to CSV."""

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


TELEMETRY_LENGTH = 7
EARTH_RADIUS_M = 6378137.0
CSV_HEADER = [
    'pc_time',
    'latitude',
    'longitude',
    'turbidity_ntu',
    'ph',
    'heading_deg',
    'do_mgl',
    'gps_fix',
]


class GPSLogger(Node):
    """Sample the latest valid telemetry at a configured interval."""

    def __init__(self):
        """Create the telemetry subscriber, output file, and timer."""
        super().__init__('gps_logger')
        self.declare_parameter('telemetry_topic', '/boat/telemetry')
        self.declare_parameter('log_interval', 1.0)
        self.declare_parameter(
            'output_directory', '/home/rodin/boat_data/gps'
        )
        self.declare_parameter('require_gps_fix', True)
        self.declare_parameter('route_yaml_enabled', True)
        self.declare_parameter('route_min_distance', 0.5)
        self.declare_parameter('route_closed_loop', False)

        interval = float(self.get_parameter('log_interval').value)
        if not math.isfinite(interval) or interval <= 0.0:
            raise ValueError('log_interval must be finite and > 0')
        output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        output_directory.mkdir(parents=True, exist_ok=True)
        self.require_gps_fix = bool(
            self.get_parameter('require_gps_fix').value
        )
        self.route_yaml_enabled = bool(
            self.get_parameter('route_yaml_enabled').value
        )
        self.route_min_distance = float(
            self.get_parameter('route_min_distance').value
        )
        self.route_closed_loop = bool(
            self.get_parameter('route_closed_loop').value
        )
        if self.route_min_distance <= 0.0:
            raise ValueError('route_min_distance must be > 0')
        session_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_name = f'gps_{session_time}.csv'
        self.csv_path = output_directory / file_name
        self.route_path = (
            output_directory / f'recorded_route_{session_time}.yaml'
        )
        self.csv_file = self.csv_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()
        self.route_waypoints = []
        if self.route_yaml_enabled:
            self._write_route_yaml()

        self.latest_telemetry = None
        self.telemetry_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter('telemetry_topic').value,
            self.telemetry_callback,
            10,
        )
        self.log_timer = self.create_timer(interval, self.log_latest)
        self.get_logger().info(
            f'GPS logger started: {self.csv_path} ({interval:g} s)'
        )
        if self.route_yaml_enabled:
            self.get_logger().info(
                f'LOS route YAML recording: {self.route_path} '
                f'(minimum {self.route_min_distance:g} m)'
            )

    def telemetry_callback(self, msg):
        """Store the latest complete, finite telemetry sample."""
        if len(msg.data) < TELEMETRY_LENGTH:
            return
        values = [float(value) for value in msg.data[:TELEMETRY_LENGTH]]
        if not all(math.isfinite(value) for value in values):
            return
        self.latest_telemetry = values

    def log_latest(self):
        """Append the latest eligible telemetry sample to CSV."""
        if self.latest_telemetry is None:
            return
        if self.require_gps_fix and self.latest_telemetry[6] <= 0.0:
            return
        self.csv_writer.writerow([
            datetime.now().isoformat(timespec='milliseconds'),
            *self.latest_telemetry,
        ])
        self.csv_file.flush()
        if self.route_yaml_enabled:
            self._append_route_waypoint(
                self.latest_telemetry[0],
                self.latest_telemetry[1],
            )

    @staticmethod
    def _distance_m(first, second):
        """Approximate WGS84 distance in metres for nearby coordinates."""
        latitude = math.radians((first[0] + second[0]) * 0.5)
        east = (
            EARTH_RADIUS_M
            * math.cos(latitude)
            * math.radians(second[1] - first[1])
        )
        north = EARTH_RADIUS_M * math.radians(second[0] - first[0])
        return math.hypot(east, north)

    def _append_route_waypoint(self, latitude, longitude):
        """Append a sufficiently separated point to the LOS route YAML."""
        point = (float(latitude), float(longitude))
        if self.route_waypoints:
            distance = self._distance_m(self.route_waypoints[-1], point)
            if distance < self.route_min_distance:
                return
        self.route_waypoints.append(point)
        self._write_route_yaml()

    def _write_route_yaml(self):
        """Atomically write all recorded points in LOS-compatible format."""
        route_data = {
            'name': self.route_path.stem,
            'coordinate_frame': 'WGS84',
            'closed_loop': self.route_closed_loop,
            'survey_grade': False,
            'source': 'boat_autonomy gps_logger',
            'warning': (
                'Recorded GNSS coordinates; verify before autonomous use.'
            ),
            'waypoints': [
                {
                    'id': index,
                    'latitude': latitude,
                    'longitude': longitude,
                }
                for index, (latitude, longitude)
                in enumerate(self.route_waypoints)
            ],
        }
        temporary_path = self.route_path.with_suffix('.yaml.tmp')
        with temporary_path.open('w', encoding='utf-8') as route_file:
            yaml.safe_dump(
                route_data,
                route_file,
                sort_keys=False,
                allow_unicode=True,
            )
            route_file.flush()
        temporary_path.replace(self.route_path)

    def destroy_node(self):
        """Flush and close the CSV file before node destruction."""
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        return super().destroy_node()


def main(args=None):
    """Run the periodic GPS logger node."""
    rclpy.init(args=args)
    node = None
    try:
        node = GPSLogger()
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
