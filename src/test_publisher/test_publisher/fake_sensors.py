import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class FakeSensors(Node):
    """Publishes a synthetic LaserScan and Odometry so the safety node
    can be tested without a simulator. A single obstacle sits straight
    ahead and closes at the declared speed."""

    def __init__(self):
        super().__init__('fake_sensors')

        self.declare_parameter('speed', 2.0)          # m/s, forward
        self.declare_parameter('start_distance', 10.0)  # m to the obstacle
        self.declare_parameter('num_beams', 181)
        self.declare_parameter('lateral_offset', 0.0)   # m, obstacle to the side

        self.speed = self.get_parameter('speed').value
        self.distance = self.get_parameter('start_distance').value
        self.num_beams = self.get_parameter('num_beams').value
        self.lateral_offset = self.get_parameter('lateral_offset').value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/ego_racecar/odom', 10)

        self.dt = 0.05                                 # 20 Hz
        self.timer = self.create_timer(self.dt, self.tick)

    def tick(self):
        # Close on the obstacle at the declared speed
        self.distance -= self.speed * self.dt
        if self.distance < 0.1:
            self.distance = 0.1

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / (self.num_beams - 1)
        scan.range_min = 0.0
        scan.range_max = 30.0
        # A point obstacle at (x, y): x ahead, y to the side.
        # Open space is inf, per the LaserScan convention.
        x = self.distance
        y = self.lateral_offset
        r = math.hypot(x, y)
        bearing = math.atan2(y, x)

        idx = round((bearing - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(self.num_beams - 1, idx))

        scan.ranges = [float('inf')] * self.num_beams
        scan.ranges[idx] = r
        self.scan_pub.publish(scan)

        # Ground truth by differentiating the range, not by reusing the node's
        # projection: r_dot = -v*x/r, so ttc = r / (v*x/r) = r^2 / (v*x)
        true_ttc = (r * r) / (self.speed * x) if self.speed > 0 else float('inf')
        beam_angle = scan.angle_min + idx * scan.angle_increment
        self.get_logger().info(
            f'x {x:.2f} m, y {y:.2f} m, range {r:.2f} m, '
            f'true TTC {true_ttc:.3f} s, '
            f'bearing {math.degrees(bearing):.2f} deg, '
            f'beam {math.degrees(beam_angle):.2f} deg')


def main():
    rclpy.init()
    rclpy.spin(FakeSensors())
    rclpy.shutdown()