#!/usr/bin/env python3
r"""
Jetson Orin Nano <-> STM32 final UART gateway for the autonomous boat.

UART physical link
------------------
STM32 USART2: PA2=TX, PA3=RX, 115200 baud, 8N1
Jetson TX -> STM PA3(RX)
Jetson RX <- STM PA2(TX)
GND       -- STM GND

Final packet protocol
---------------------
STM32 -> Jetson (10 Hz):
  $TEL,<lat>,<lon>,<turbidity_ntu>,<ph>,<heading_deg>,<do_mgl>,<gps_fix>*<XOR>\r\n

Jetson -> STM32:
  $CMD,<direction_deg>,<output>*<XOR>\r\n
  direction_deg: -180 .. +180
  output       : 0 .. 10

ROS2 interface
--------------
Publish: /boat/telemetry  std_msgs/msg/Float64MultiArray
  data[0] = latitude
  data[1] = longitude
  data[2] = turbidity_ntu
  data[3] = ph
  data[4] = heading_deg
  data[5] = do_mgl
  data[6] = gps_fix

Subscribe: /boat/cmd     std_msgs/msg/Float64MultiArray
  data[0] = direction_deg
  data[1] = output

Example command test:
  ros2 topic pub -r 10 /boat/cmd std_msgs/msg/Float64MultiArray \
    "{data: [-20.0, 5.0]}"

Run:
  source /opt/ros/humble/setup.bash
  python3 jetson_boat_uart_final.py --ros-args -p port:=/dev/ttyTHS1
"""

from __future__ import annotations

import csv
import math
import threading
import time
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial 필요: python3 -m pip install pyserial") from exc

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Float64MultiArray
except ImportError as exc:
    raise SystemExit(
        "ROS2 환경을 먼저 source 하세요: source /opt/ros/humble/setup.bash"
    ) from exc


# ============================================================================
# Data structures
# ============================================================================

@dataclass(frozen=True)
class Telemetry:
    latitude: float = 0.0
    longitude: float = 0.0
    turbidity_ntu: float = 0.0
    ph: float = 0.0
    heading_deg: float = 0.0
    do_mgl: float = 0.0
    gps_fix: int = 0
    received_at: float = 0.0  # time.monotonic()


@dataclass(frozen=True)
class MotionCommand:
    direction_deg: float = 0.0
    output: float = 0.0

    def validate(self) -> None:
        if not math.isfinite(self.direction_deg):
            raise ValueError("direction is NaN/Inf")
        if not math.isfinite(self.output):
            raise ValueError("output is NaN/Inf")
        if not -180.0 <= self.direction_deg <= 180.0:
            raise ValueError("direction must be -180..180 deg")
        if not 0.0 <= self.output <= 10.0:
            raise ValueError("output must be 0..10")


# ============================================================================
# Packet protocol - identical checksum rule on both boards
# ============================================================================

def xor_checksum(body: str) -> int:
    cs = 0
    for b in body.encode("ascii"):
        cs ^= b
    return cs


def decode_packet(line: str) -> str | None:
    """Validate '$BODY*CS' and return BODY."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None

    body, cs_text = line[1:].rsplit("*", 1)
    if len(cs_text) < 2:
        return None

    try:
        received_cs = int(cs_text[:2], 16)
    except ValueError:
        return None

    if xor_checksum(body) != received_cs:
        return None

    return body


def parse_telemetry(line: str) -> Telemetry | None:
    body = decode_packet(line)
    if body is None:
        return None

    f = body.split(",")

    # TEL,lat,lon,turbidity,ph,heading,do,gps_fix
    if len(f) != 8 or f[0] != "TEL":
        return None

    try:
        tel = Telemetry(
            latitude=float(f[1]),
            longitude=float(f[2]),
            turbidity_ntu=float(f[3]),
            ph=float(f[4]),
            heading_deg=float(f[5]),
            do_mgl=float(f[6]),
            gps_fix=int(f[7]),
            received_at=time.monotonic(),
        )
    except ValueError:
        return None

    values = (
        tel.latitude,
        tel.longitude,
        tel.turbidity_ntu,
        tel.ph,
        tel.heading_deg,
        tel.do_mgl,
    )
    if not all(math.isfinite(v) for v in values):
        return None

    # Basic sanity filtering. gps_fix==0 is allowed; it simply means no valid fix.
    if not -90.0 <= tel.latitude <= 90.0:
        return None
    if not -180.0 <= tel.longitude <= 180.0:
        return None
    if not 0.0 <= tel.ph <= 14.0:
        return None
    if not 0.0 <= tel.heading_deg < 360.0:
        return None
    if tel.turbidity_ntu < 0.0 or tel.do_mgl < 0.0:
        return None

    return tel


def make_command_packet(cmd: MotionCommand) -> bytes:
    cmd.validate()
    body = f"CMD,{cmd.direction_deg:.2f},{cmd.output:.2f}"
    return f"${body}*{xor_checksum(body):02X}\r\n".encode("ascii")


# ============================================================================
# Serial link
# ============================================================================

class STM32Link:
    def __init__(self, port: str, baudrate: int) -> None:
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.2,
        )

        self._tel_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._latest = Telemetry()
        self.bad_packet_count = 0
        self.good_packet_count = 0

    def start(self) -> None:
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="stm32-uart-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def close(self) -> None:
        # Try to stop the boat before closing UART.
        try:
            self.send_command(MotionCommand(0.0, 0.0))
        except Exception:
            pass

        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)

        if self.ser.is_open:
            self.ser.close()

    def snapshot(self) -> Telemetry:
        with self._tel_lock:
            return replace(self._latest)

    def send_command(self, cmd: MotionCommand) -> None:
        packet = make_command_packet(cmd)
        with self._tx_lock:
            self.ser.write(packet)
            self.ser.flush()

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                return

            if not raw:
                continue

            line = raw.decode("ascii", errors="ignore")
            tel = parse_telemetry(line)

            if tel is None:
                self.bad_packet_count += 1
                continue

            self.good_packet_count += 1
            with self._tel_lock:
                self._latest = tel


# ============================================================================
# ROS2 node
# ============================================================================

class BoatUartNode(Node):
    def __init__(self) -> None:
        super().__init__("boat_uart_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("loop_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("telemetry_timeout", 0.5)
        self.declare_parameter("csv_path", "")

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        loop_rate = float(self.get_parameter("loop_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)
        csv_path = str(self.get_parameter("csv_path").value)

        if loop_rate <= 0.0:
            raise ValueError("loop_rate must be > 0")

        self.link = STM32Link(port, baudrate)
        self.link.start()

        self.telemetry_pub = self.create_publisher(
            Float64MultiArray,
            "/boat/telemetry",
            10,
        )
        self.telemetry_age_pub = self.create_publisher(
            Float32,
            "/boat/telemetry_age",
            10,
        )
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            "/boat/cmd",
            self._command_callback,
            10,
        )

        self.latest_command = MotionCommand(0.0, 0.0)
        self.last_command_at = 0.0
        self.last_status_print = 0.0

        self.csv_file = None
        self.csv_writer = None
        self.last_csv_tel_time = 0.0
        if csv_path:
            path = Path(csv_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open("a", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            if path.stat().st_size == 0:
                self.csv_writer.writerow([
                    "pc_time",
                    "latitude",
                    "longitude",
                    "turbidity_ntu",
                    "ph",
                    "heading_deg",
                    "do_mgl",
                    "gps_fix",
                ])
                self.csv_file.flush()

        self.timer = self.create_timer(1.0 / loop_rate, self._timer_callback)

        self.get_logger().info(
            f"UART started: {port}, {baudrate} baud, 8N1"
        )
        self.get_logger().info(
            "STM->Jetson: TEL(lat,lon,NTU,pH,heading,DO,fix) / "
            "Jetson->STM: CMD(direction,output)"
        )

    def _command_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning("/boat/cmd needs [direction, output]")
            self.latest_command = MotionCommand(0.0, 0.0)
            self.last_command_at = 0.0
            return

        cmd = MotionCommand(
            direction_deg=float(msg.data[0]),
            output=float(msg.data[1]),
        )

        try:
            cmd.validate()
        except ValueError as exc:
            self.get_logger().warning(f"Invalid /boat/cmd: {exc}")
            self.latest_command = MotionCommand(0.0, 0.0)
            self.last_command_at = 0.0
            return

        self.latest_command = cmd
        self.last_command_at = time.monotonic()

    def _timer_callback(self) -> None:
        now = time.monotonic()
        tel = self.link.snapshot()

        # 1) Publish STM telemetry to ROS2.
        msg = Float64MultiArray()
        msg.data = [
            tel.latitude,
            tel.longitude,
            tel.turbidity_ntu,
            tel.ph,
            tel.heading_deg,
            tel.do_mgl,
            float(tel.gps_fix),
        ]
        self.telemetry_pub.publish(msg)
        age_msg = Float32()
        age_msg.data = float(
            now - tel.received_at if tel.received_at > 0.0 else -1.0
        )
        self.telemetry_age_pub.publish(age_msg)

        # 2) Optional CSV logging. Write only once per newly received TEL packet.
        if (
            self.csv_writer is not None
            and tel.received_at > self.last_csv_tel_time
        ):
            self.csv_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"),
                f"{tel.latitude:.7f}",
                f"{tel.longitude:.7f}",
                f"{tel.turbidity_ntu:.2f}",
                f"{tel.ph:.2f}",
                f"{tel.heading_deg:.2f}",
                f"{tel.do_mgl:.2f}",
                tel.gps_fix,
            ])
            self.csv_file.flush()
            self.last_csv_tel_time = tel.received_at

        # 3) Watchdogs. If either the control command or STM telemetry is stale,
        #    keep sending STOP. STM also has its own 500 ms command watchdog.
        telemetry_fresh = (
            tel.received_at > 0.0
            and (now - tel.received_at) <= self.telemetry_timeout
        )
        command_fresh = (
            self.last_command_at > 0.0
            and (now - self.last_command_at) <= self.command_timeout
        )

        if telemetry_fresh and command_fresh:
            cmd_to_send = self.latest_command
        else:
            cmd_to_send = MotionCommand(0.0, 0.0)

        try:
            self.link.send_command(cmd_to_send)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"UART TX error: {exc}")

        # 4) Human-readable status once per second.
        if now - self.last_status_print >= 1.0:
            age = (now - tel.received_at) if tel.received_at > 0.0 else float("inf")
            self.get_logger().info(
                f"GPS=({tel.latitude:.7f},{tel.longitude:.7f}) "
                f"fix={tel.gps_fix} HDG={tel.heading_deg:.1f}deg "
                f"NTU={tel.turbidity_ntu:.2f} pH={tel.ph:.2f} "
                f"DO={tel.do_mgl:.2f}mg/L TEL_age={age:.2f}s "
                f"CMD=({cmd_to_send.direction_deg:.1f},{cmd_to_send.output:.1f})"
            )
            self.last_status_print = now

    def destroy_node(self) -> bool:
        try:
            self.link.send_command(MotionCommand(0.0, 0.0))
        except Exception:
            pass
        self.link.close()
        if self.csv_file is not None:
            self.csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BoatUartNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        raise SystemExit(f"UART open/read/write failed: {exc}") from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
