"""K9 six-channel downward VL53L0X cliff monitor (ROS 2 Jazzy).

Outputs are advisory to the independent velocity safety gateway. The gateway
MUST enforce its own message-age watchdog: a dead node cannot publish unsafe.

State codes: CLEAR=0, WARNING=1, CLIFF=2, UNKNOWN=3.
Sensor array order: front_left, front_centre, front_right,
                    rear_left, rear_centre, rear_right.
"""

from dataclasses import dataclass
from math import isfinite, nan

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, UInt8MultiArray

CLEAR, WARNING, CLIFF, UNKNOWN = 0, 1, 2, 3
SENSOR_NAMES = (
    'front_left', 'front_centre', 'front_right',
    'rear_left', 'rear_centre', 'rear_right',
)
DEFAULTS = (
    (0, 56.8, 1.66), (1, 37.4, 3.51), (2, 36.7, 1.99),
    (3, 46.5, 3.06), (4, 91.5, 2.17), (5, 48.6, 2.20),
)


def thresholds(mean_mm, stddev_mm):
    if not (isfinite(mean_mm) and isfinite(stddev_mm)) or stddev_mm < 0:
        raise ValueError('Invalid cliff calibration')
    return (mean_mm + max(3.0 * stddev_mm, 10.0),
            mean_mm + max(5.0 * stddev_mm, 20.0))


@dataclass
class Sensor:
    name: str
    channel: int
    warning_mm: float
    cliff_mm: float
    driver: object = None
    state: int = UNKNOWN
    clear_count: int = 0

    def update(self, distance_mm, clear_samples):
        """Latch stop hazards until clear readings have been consistent."""
        if distance_mm is None or not isfinite(distance_mm) or distance_mm <= 0:
            self.state = UNKNOWN
            self.clear_count = 0
            return
        if distance_mm >= self.cliff_mm:
            self.state = CLIFF
            self.clear_count = 0
            return
        if distance_mm >= self.warning_mm:
            # A previous stop is not released while a warning persists.
            if self.state not in (CLIFF, UNKNOWN):
                self.state = WARNING
            self.clear_count = 0
            return
        self.clear_count += 1
        if self.state in (CLIFF, UNKNOWN) and self.clear_count < clear_samples:
            return
        self.state = CLEAR


class CliffDetector(Node):
    def __init__(self):
        super().__init__('cliff_detector')
        self.declare_parameter('sample_rate_hz', 5.0)
        self.declare_parameter('mux_address', 0x70)
        self.declare_parameter('clear_samples', 3)
        self.declare_parameter('dry_run', False)
        rate = float(self.get_parameter('sample_rate_hz').value)
        self.clear_samples = int(self.get_parameter('clear_samples').value)
        if not (0 < rate <= 20) or self.clear_samples < 1:
            raise ValueError('Invalid sample_rate_hz or clear_samples')

        self.sensors = []
        for name, (channel_default, mean_default, sd_default) in zip(SENSOR_NAMES, DEFAULTS):
            self.declare_parameter(f'{name}.channel', channel_default)
            self.declare_parameter(f'{name}.mean_mm', mean_default)
            self.declare_parameter(f'{name}.stddev_mm', sd_default)
            channel = int(self.get_parameter(f'{name}.channel').value)
            mean = float(self.get_parameter(f'{name}.mean_mm').value)
            sd = float(self.get_parameter(f'{name}.stddev_mm').value)
            warn, cliff = thresholds(mean, sd)
            if channel < 0 or channel > 7:
                raise ValueError(f'Invalid mux channel for {name}: {channel}')
            self.sensors.append(Sensor(name, channel, warn, cliff))
            self.get_logger().info(f'{name} CH{channel}: warn >= {warn:.2f} mm; cliff >= {cliff:.2f} mm')
        if len(set(s.channel for s in self.sensors)) != len(self.sensors):
            raise ValueError('Duplicate I2C mux channels')

        self.distances_pub = self.create_publisher(Float32MultiArray, '/k9/cliff/distances_mm', 10)
        self.states_pub = self.create_publisher(UInt8MultiArray, '/k9/cliff/states', 10)
        self.front_stop_pub = self.create_publisher(Bool, '/k9/cliff/front_stop', 10)
        self.rear_stop_pub = self.create_publisher(Bool, '/k9/cliff/rear_stop', 10)
        self.front_warn_pub = self.create_publisher(Bool, '/k9/cliff/front_warning', 10)
        self.rear_warn_pub = self.create_publisher(Bool, '/k9/cliff/rear_warning', 10)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self._mux = None
        self._open_hardware()
        self._last_status = None
        self.create_timer(1.0 / rate, self._tick)

    def _open_hardware(self):
        if self.dry_run:
            self.get_logger().warning('dry_run: no hardware; all sensors UNKNOWN (unsafe)')
            return
        # Use the same Blinka + Adafruit mux stack as the working sensor test.
        import board
        import adafruit_tca9548a
        import adafruit_vl53l0x
        i2c = board.I2C()
        self._mux = adafruit_tca9548a.TCA9548A(
            i2c, address=int(self.get_parameter('mux_address').value))
        for sensor in self.sensors:
            try:
                sensor.driver = adafruit_vl53l0x.VL53L0X(self._mux[sensor.channel])
            except Exception as exc:
                self.get_logger().error(f'Init {sensor.name} failed: {exc}')
                sensor.driver = None

    def _tick(self):
        readings = []
        for sensor in self.sensors:
            reading = None
            if sensor.driver is not None:
                try:
                    reading = float(sensor.driver.range)  # mm
                except Exception as exc:
                    # Failure is not an all-clear; continue polling other sensors.
                    self.get_logger().warning(f'{sensor.name} read failed: {exc}', throttle_duration_sec=5.0)
            readings.append(reading if reading is not None else nan)
            sensor.update(reading, self.clear_samples)

        states = [s.state for s in self.sensors]
        self.distances_pub.publish(Float32MultiArray(data=readings))
        self.states_pub.publish(UInt8MultiArray(data=states))
        front, rear = states[:3], states[3:]
        # UNKNOWN is deliberately a stop, not a clear.
        front_stop = any(s in (CLIFF, UNKNOWN) for s in front)
        rear_stop = any(s in (CLIFF, UNKNOWN) for s in rear)
        front_warning = any(s == WARNING for s in front)
        rear_warning = any(s == WARNING for s in rear)
        self.front_stop_pub.publish(Bool(data=front_stop))
        self.rear_stop_pub.publish(Bool(data=rear_stop))
        self.front_warn_pub.publish(Bool(data=front_warning))
        self.rear_warn_pub.publish(Bool(data=rear_warning))

        status = tuple(states)
        if status != self._last_status:
            self.get_logger().info('Cliff states: ' + ', '.join(
                f'{name}={state}' for name, state in zip(SENSOR_NAMES, states)))
            self._last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CliffDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
