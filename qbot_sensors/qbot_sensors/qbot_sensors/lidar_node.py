"""ROS 2 node: publishes the QBot Platform lidar as sensor_msgs/LaserScan on /scan."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from qbot_sensors.quanser_hw import QBotPlatformLidar


class LidarNode(Node):
    def __init__(self):
        super().__init__('qbot_lidar')
        self.declare_parameter('frame_id', 'lidar')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('num_measurements', 1680)
        self.declare_parameter('num_bins', 720)          # 0.5 deg resolution
        self.declare_parameter('range_min', 0.10)        # m
        self.declare_parameter('range_max', 12.0)        # m
        # Quanser headings may be clockwise; ROS expects counter-clockwise.
        # Check in RViz and flip these if the scan looks mirrored / rotated.
        self.declare_parameter('reverse_direction', True)
        self.declare_parameter('angle_offset_deg', 0.0)

        gp = lambda n: self.get_parameter(n).value
        self.frame_id = gp('frame_id')
        self.n_bins = int(gp('num_bins'))
        self.range_min = float(gp('range_min'))
        self.range_max = float(gp('range_max'))
        self.reverse = bool(gp('reverse_direction'))
        self.offset = math.radians(float(gp('angle_offset_deg')))
        self.inc = 2.0 * math.pi / self.n_bins

        self.pub = self.create_publisher(LaserScan, gp('topic'), qos_profile_sensor_data)

        self.get_logger().info('Opening QBot Platform lidar...')
        self.lidar = QBotPlatformLidar(numMeasurements=int(gp('num_measurements')))
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.loop)   # poll at 100 Hz
        self.get_logger().info(f"Publishing LaserScan on {gp('topic')}")

    def loop(self):
        if not self.lidar.read():
            return
        now = self.get_clock().now()
        dist = np.asarray(self.lidar.distances, dtype=np.float64).ravel()
        ang = np.asarray(self.lidar.angles, dtype=np.float64).ravel()

        valid = (dist >= self.range_min) & (dist <= self.range_max)
        dist, ang = dist[valid], ang[valid]
        if self.reverse:
            ang = -ang
        ang = np.mod(ang + self.offset, 2.0 * math.pi)
        idx = np.minimum((ang / self.inc).astype(int), self.n_bins - 1)

        ranges = np.full(self.n_bins, np.inf)
        np.minimum.at(ranges, idx, dist)   # keep the closest hit per bin

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - self.inc
        msg.angle_increment = self.inc
        dt = (now - self.last_time).nanoseconds * 1e-9
        msg.scan_time = float(dt) if dt < 1.0 else 0.1
        msg.time_increment = msg.scan_time / self.n_bins
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(msg)
        self.last_time = now

    def destroy_node(self):
        try:
            self.lidar.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
