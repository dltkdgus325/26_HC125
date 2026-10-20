#!/usr/bin/env python3
"""Arbitrate LOS global control and camera-based trash approach control."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String


def clamp(value, low, high):
    """Clamp a value to an inclusive range."""
    return max(low, min(high, value))


class TrashController(Node):
    """Select global LOS control or local visual approach control."""

    GLOBAL = 'GLOBAL'
    APPROACH = 'APPROACH'
    COLLECT = 'COLLECT'

    def __init__(self):
        """Create subscriptions, command publishers, and controller state."""
        super().__init__('trash_controller')
        defaults = {
            'target_topic': '/trash/target',
            'target_state_topic': '/trash/target_state',
            'global_command_topic': '/autonomy/global_cmd',
            'final_command_topic': '/boat/cmd',
            'control_config_topic': '/autonomy/trash_control_config',
            'mode_topic': '/autonomy/mode', 'control_rate': 20.0,
            'global_command_timeout': 0.5, 'target_timeout': 0.5,
            'steering_kp': 7.0, 'steering_kd': 1.0,
            'error_filter_alpha': 0.15,
            'derivative_filter_alpha': 0.20,
            'max_steering_rate_deg_per_sec': 5.0,
            'steering_sign': 1.0, 'max_steering_deg': 15.0,
            'approach_output': 3.0, 'minimum_approach_output': 1.5,
            'slow_area_ratio': 0.003,
            'full_slow_area_ratio': 0.010,
            'center_deadband_min': 0.03,
            'center_deadband_max': 0.12,
            'center_deadband_bbox_gain': 0.8,
            'bottom_deadband_start_ratio': 0.75,
            'bottom_deadband_max': 0.35,
            'bottom_steering_scale': 0.35,
            'bottom_steering_decay_deg_per_sec': 20.0,
            'bottom_output_max': 1.5,
            'max_output_accel_per_sec': 0.8,
            'max_output_decel_per_sec': 2.0,
            'target_edge_margin_ratio': 0.15,
            'edge_min_steering_deg': 5.0,
            'edge_output_scale': 0.70,
            'temp_lost_steering_decay_deg_per_sec': 20.0,
            'temp_lost_output': 1.0,
            'collection_lock_min_duration': 3.0,
            'collection_drive_duration': 3.0,
            'collection_output': 1.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value
        for name, default in defaults.items():
            if isinstance(default, float):
                setattr(self, name, float(get(name)))
        self._validate_parameters()

        self.mode = self.GLOBAL
        self.global_command = None
        self.global_time = None
        self.target = None
        self.target_time = None
        self.target_filter_state = 'NONE'
        self.locked_visible_since = None
        self.qualified_target_lost = False
        self.collection_started_at = None
        self.approach_initialized = False
        self.filtered_error = 0.0
        self.previous_filtered_error = 0.0
        self.filtered_error_rate = 0.0
        self.previous_direction = 0.0
        self.previous_output = self.minimum_approach_output
        self.last_approach_time = None

        self.command_pub = self.create_publisher(
            Float64MultiArray, str(get('final_command_topic')), 10
        )
        self.mode_pub = self.create_publisher(
            String, str(get('mode_topic')), 10
        )
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.config_pub = self.create_publisher(
            Float64MultiArray, str(get('control_config_topic')), config_qos
        )
        config_msg = Float64MultiArray()
        config_msg.data = [
            self.center_deadband_min,
            self.center_deadband_max,
            self.center_deadband_bbox_gain,
            self.bottom_deadband_start_ratio,
            self.bottom_deadband_max,
        ]
        self.config_pub.publish(config_msg)
        self.global_sub = self.create_subscription(
            Float64MultiArray, str(get('global_command_topic')),
            self.global_callback, 10
        )
        self.target_sub = self.create_subscription(
            Float64MultiArray, str(get('target_topic')),
            self.target_callback, 10
        )
        self.target_state_sub = self.create_subscription(
            String, str(get('target_state_topic')),
            self.target_state_callback, 10
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )
        self.get_logger().info('Trash controller started in GLOBAL mode')

    def _validate_parameters(self):
        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be > 0')
        if not 0.0 < self.slow_area_ratio < self.full_slow_area_ratio <= 1.0:
            raise ValueError('area ratios must satisfy 0 < slow < stop <= 1')
        if self.steering_kp < 0.0 or self.steering_kd < 0.0:
            raise ValueError('steering gains must be >= 0')
        if not 0.0 < self.error_filter_alpha <= 1.0:
            raise ValueError('error_filter_alpha must be in (0, 1]')
        if not 0.0 < self.derivative_filter_alpha <= 1.0:
            raise ValueError('derivative_filter_alpha must be in (0, 1]')
        if self.max_steering_rate_deg_per_sec <= 0.0:
            raise ValueError('max steering rate must be > 0')
        if not 0.0 <= self.approach_output <= 10.0:
            raise ValueError('approach_output must be in [0, 10]')
        if not 0.0 <= self.minimum_approach_output <= 10.0:
            raise ValueError('minimum_approach_output must be in [0, 10]')
        if self.minimum_approach_output > self.approach_output:
            raise ValueError('minimum output must be <= approach output')
        if not (0.0 <= self.center_deadband_min
                <= self.center_deadband_max < 1.0):
            raise ValueError('deadbands must satisfy 0 <= min <= max < 1')
        if self.center_deadband_bbox_gain < 0.0:
            raise ValueError('center_deadband_bbox_gain must be >= 0')
        if not 0.0 < self.bottom_deadband_start_ratio < 1.0:
            raise ValueError('bottom_deadband_start_ratio must be in (0, 1)')
        if not self.center_deadband_max <= self.bottom_deadband_max < 1.0:
            raise ValueError(
                'bottom_deadband_max must be in [center max, 1)'
            )
        if not 0.0 < self.bottom_steering_scale <= 1.0:
            raise ValueError('bottom_steering_scale must be in (0, 1]')
        if self.bottom_steering_decay_deg_per_sec <= 0.0:
            raise ValueError('bottom steering decay must be > 0')
        if not 0.0 <= self.bottom_output_max <= self.approach_output:
            raise ValueError(
                'bottom_output_max must be in [0, approach_output]'
            )
        if (self.max_output_accel_per_sec <= 0.0
                or self.max_output_decel_per_sec <= 0.0):
            raise ValueError('output rates must be > 0')
        if not 0.0 < self.target_edge_margin_ratio < 0.5:
            raise ValueError('target_edge_margin_ratio must be in (0, 0.5)')
        if not 0.0 <= self.edge_min_steering_deg <= self.max_steering_deg:
            raise ValueError(
                'edge_min_steering_deg must be in [0, max steering]'
            )
        if not 0.0 < self.edge_output_scale <= 1.0:
            raise ValueError('edge_output_scale must be in (0, 1]')
        if self.temp_lost_steering_decay_deg_per_sec <= 0.0:
            raise ValueError('temp lost steering decay must be > 0')
        if not 0.0 <= self.temp_lost_output <= 10.0:
            raise ValueError('temp_lost_output must be in [0, 10]')
        if self.collection_lock_min_duration <= 0.0:
            raise ValueError('collection_lock_min_duration must be > 0')
        if self.collection_drive_duration <= 0.0:
            raise ValueError('collection_drive_duration must be > 0')
        if not 0.0 <= self.collection_output <= 10.0:
            raise ValueError('collection_output must be in [0, 10]')

    def global_callback(self, msg):
        """Store the latest valid command produced by the LOS controller."""
        if len(msg.data) < 2:
            return
        values = tuple(map(float, msg.data[:2]))
        if (all(math.isfinite(v) for v in values)
                and -180.0 <= values[0] <= 180.0
                and 0.0 <= values[1] <= 10.0):
            self.global_command = values
            self.global_time = time.monotonic()

    def target_callback(self, msg):
        """Store the latest valid detected target."""
        if len(msg.data) < 10:
            return
        values = list(map(float, msg.data[:10]))
        if not all(math.isfinite(v) for v in values):
            return
        if values[0] < 0.5 or values[7] <= 0.0 or values[8] <= 0.0:
            self.target = None
            return
        self.target = values
        self.target_time = time.monotonic()

    def target_state_callback(self, msg):
        """Track continuous visible lock time for collection commitment."""
        state = str(msg.data).upper()
        if state not in ('NONE', 'CANDIDATE', 'LOCKED', 'TEMP_LOST'):
            return
        now = time.monotonic()
        previous = self.target_filter_state
        if state == 'LOCKED':
            if previous != 'LOCKED':
                self.locked_visible_since = now
                self.qualified_target_lost = False
        elif previous == 'LOCKED':
            visible_duration = (
                now - self.locked_visible_since
                if self.locked_visible_since is not None else 0.0
            )
            self.qualified_target_lost = (
                state == 'TEMP_LOST'
                and visible_duration >= self.collection_lock_min_duration
            )
            self.locked_visible_since = None
        elif state in ('NONE', 'CANDIDATE') and self.mode != self.COLLECT:
            self.qualified_target_lost = False
        self.target_filter_state = state

    def control_loop(self):
        """Update the state machine and publish exactly one final command."""
        now = time.monotonic()
        target = self._fresh_target(now)
        if self.mode == self.GLOBAL:
            if target is not None:
                self._set_mode(self.APPROACH)
        elif self.mode == self.APPROACH:
            if (self.target_filter_state == 'TEMP_LOST'
                    and self.qualified_target_lost):
                self.collection_started_at = now
                self._set_mode(self.COLLECT)
            elif target is None:
                self._set_mode(self.GLOBAL)
        elif self.mode == self.COLLECT:
            if (self.collection_started_at is None
                    or now - self.collection_started_at
                    >= self.collection_drive_duration):
                self.collection_started_at = None
                self.qualified_target_lost = False
                self._set_mode(self.GLOBAL)

        if self.mode == self.GLOBAL:
            self._publish_global(now)
        elif self.mode == self.APPROACH and target is not None:
            if self.target_filter_state == 'TEMP_LOST':
                self._publish_temp_lost(now)
            else:
                self._publish_approach(target, now)
        elif self.mode == self.COLLECT:
            self._publish(0.0, self.collection_output)
        else:
            self._publish(0.0, 0.0)
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

    def _fresh_target(self, now):
        if self.target is None or self.target_time is None:
            return None
        if now - self.target_time <= self.target_timeout:
            return self.target
        return None

    def _target_geometry(self, target):
        error = (target[3] - target[7] / 2.0) / (target[7] / 2.0)
        area = target[5] * target[6] / (target[7] * target[8])
        return clamp(error, -1.0, 1.0), area

    def _publish_approach(self, target, now):
        """Publish rate-limited PD steering and unified approach output."""
        raw_error, area = self._target_geometry(target)
        bbox_width_ratio = target[5] / target[7]
        bbox_bottom_ratio = clamp(
            (target[4] + target[6] / 2.0) / target[8], 0.0, 1.0
        )
        bottom_progress = clamp(
            (bbox_bottom_ratio - self.bottom_deadband_start_ratio)
            / (1.0 - self.bottom_deadband_start_ratio),
            0.0, 1.0,
        )
        deadband = clamp(
            self.center_deadband_min
            + self.center_deadband_bbox_gain * bbox_width_ratio,
            self.center_deadband_min, self.center_deadband_max
        )
        deadband += bottom_progress * (
            self.bottom_deadband_max - deadband
        )
        if not self.approach_initialized:
            self.filtered_error = raw_error
            effective_error = self._apply_deadband(raw_error, deadband)
            self.previous_filtered_error = self.filtered_error
            self.filtered_error_rate = 0.0
            self.previous_direction = 0.0
            self.last_approach_time = now
            self.approach_initialized = True
            dt = 1.0 / self.control_rate
        else:
            dt = clamp(now - self.last_approach_time, 1e-3, 0.2)
            self.last_approach_time = now
            alpha = self.error_filter_alpha
            self.filtered_error = (
                (1.0 - alpha) * self.filtered_error
                + alpha * raw_error
            )
            effective_error = self._apply_deadband(
                self.filtered_error, deadband
            )
        measured_error_rate = (
            self.filtered_error - self.previous_filtered_error
        ) / dt
        derivative_alpha = self.derivative_filter_alpha
        self.filtered_error_rate = (
            (1.0 - derivative_alpha) * self.filtered_error_rate
            + derivative_alpha * measured_error_rate
        )
        self.previous_filtered_error = self.filtered_error
        requested_direction = clamp(
            self.steering_sign * (
                self.steering_kp * effective_error
                + self.steering_kd * self.filtered_error_rate
                * (1.0 - bottom_progress)
            ),
            -self.max_steering_deg, self.max_steering_deg
        )
        alignment_direction = self.steering_sign * effective_error
        if alignment_direction == 0.0:
            requested_direction = 0.0
        elif requested_direction * alignment_direction < 0.0:
            requested_direction = 0.0
        edge_threshold = 1.0 - 2.0 * self.target_edge_margin_ratio
        target_near_edge = abs(raw_error) >= edge_threshold
        if target_near_edge and bottom_progress < 1.0:
            inward_direction = self.steering_sign * raw_error
            requested_direction = math.copysign(
                max(
                    abs(requested_direction),
                    self.edge_min_steering_deg * (1.0 - bottom_progress),
                ),
                inward_direction,
            )
        steering_limit = self.max_steering_deg * (
            1.0 - bottom_progress * (1.0 - self.bottom_steering_scale)
        )
        requested_direction = clamp(
            requested_direction, -steering_limit, steering_limit
        )
        steering_rate = self.max_steering_rate_deg_per_sec
        reducing_steering = (
            abs(requested_direction) < abs(self.previous_direction)
            or requested_direction * self.previous_direction <= 0.0
        )
        if reducing_steering:
            steering_rate += bottom_progress * (
                self.bottom_steering_decay_deg_per_sec - steering_rate
            )
        steering_change = steering_rate * dt
        direction = clamp(
            requested_direction,
            self.previous_direction - steering_change,
            self.previous_direction + steering_change
        )
        self.previous_direction = direction

        if area <= self.slow_area_ratio:
            distance_scale = 1.0
        else:
            distance_scale = clamp(
                (self.full_slow_area_ratio - area)
                / (self.full_slow_area_ratio - self.slow_area_ratio),
                0.0, 1.0
            )
        alignment_scale = clamp(
            1.0 - abs(effective_error) / (1.0 - deadband), 0.0, 1.0
        )
        requested_output = self.minimum_approach_output + (
            self.approach_output - self.minimum_approach_output
        ) * distance_scale * alignment_scale
        if target_near_edge:
            requested_output *= self.edge_output_scale
        bottom_output_limit = self.approach_output + bottom_progress * (
            self.bottom_output_max - self.approach_output
        )
        requested_output = min(requested_output, bottom_output_limit)
        if not hasattr(self, '_output_rate_initialized'):
            self.previous_output = requested_output
            self._output_rate_initialized = True
        increase = self.max_output_accel_per_sec * dt
        decrease = self.max_output_decel_per_sec * dt
        output = clamp(
            requested_output,
            self.previous_output - decrease,
            self.previous_output + increase
        )
        self.previous_output = output
        self._publish(direction, output)

    def _publish_temp_lost(self, now):
        """Decay stale steering instead of following the last bounding box."""
        if self.last_approach_time is None:
            dt = 1.0 / self.control_rate
        else:
            dt = clamp(now - self.last_approach_time, 1e-3, 0.2)
        self.last_approach_time = now
        decay = self.temp_lost_steering_decay_deg_per_sec * dt
        if abs(self.previous_direction) <= decay:
            direction = 0.0
        else:
            direction = self.previous_direction - math.copysign(
                decay, self.previous_direction
            )
        self.previous_direction = direction
        self._publish(direction, self.temp_lost_output)

    @staticmethod
    def _apply_deadband(error, deadband):
        if abs(error) <= deadband:
            return 0.0
        return math.copysign(abs(error) - deadband, error)

    def _publish_global(self, now):
        if (self.global_command is None or self.global_time is None
                or now - self.global_time > self.global_command_timeout):
            self._publish(0.0, 0.0)
        else:
            self._publish(*self.global_command)

    def _set_mode(self, mode):
        self.get_logger().info(f'Mode: {self.mode} -> {mode}')
        if mode == self.COLLECT:
            self.get_logger().info(
                'Collection drive committed: straight for '
                f'{self.collection_drive_duration:.1f}s at output '
                f'{self.collection_output:.2f}'
            )
        self.mode = mode
        self.approach_initialized = False
        self.last_approach_time = None
        if hasattr(self, '_output_rate_initialized'):
            del self._output_rate_initialized

    def _publish(self, direction, output):
        msg = Float64MultiArray()
        msg.data = [float(direction), float(output)]
        self.command_pub.publish(msg)

    def destroy_node(self):
        """Publish a final stop command before node destruction."""
        if rclpy.ok():
            try:
                self._publish(0.0, 0.0)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    """Run the trash command arbitration node."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrashController()
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
