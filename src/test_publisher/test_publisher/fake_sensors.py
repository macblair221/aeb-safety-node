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
        self.declare_parameter('num_beams', 180)

        self.speed = self.get_parameter('speed').value
        self.distance = self.get_parameter('start_distance').value
        self.num_beams = self.get_parameter('num_beams').value

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
        # Flat wall perpendicular to the vehicle: range grows as 1/cos(angle)
        scan.ranges = [
            self.distance / math.cos(scan.angle_min + i * scan.angle_increment)
            for i in range(self.num_beams)
        ]
        self.scan_pub.publish(scan)

        true_ttc = self.distance / self.speed if self.speed > 0 else float('inf')
        self.get_logger().info(
            f'distance {self.distance:.2f} m, speed {self.speed:.2f} m/s, '
            f'true TTC {true_ttc:.2f} s')


def main():
    rclpy.init()
    rclpy.spin(FakeSensors())
    rclpy.shutdown()