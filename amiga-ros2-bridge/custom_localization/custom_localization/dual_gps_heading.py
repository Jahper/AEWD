import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
from tf2_ros import Buffer, TransformListener, LookupException


def yaw_to_quaternion(yaw_rad):
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class DualGPSHeading(Node):

    def __init__(self):
        super().__init__('dual_gps_heading')

        self.declare_parameter('left_gps_frame', 'left_ublox_link')
        self.declare_parameter('right_gps_frame', 'right_ublox_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_baseline_m', 0.3)

        self.left_frame = self.get_parameter('left_gps_frame').value
        self.right_frame = self.get_parameter('right_gps_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_baseline = self.get_parameter('min_baseline_m').value

        self.baseline_offset_rad = None
        self.left_fix = None
        self.right_fix = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(1.0, self.try_resolve_baseline)

        self.heading_pub = self.create_publisher(Imu, '/gps/heading', 10)

        self.create_subscription(NavSatFix, '/left/fix', self.left_callback, 10)
        self.create_subscription(NavSatFix, '/right/fix', self.right_callback, 10)

        self.get_logger().info('DualGPSHeading node started, waiting for TF...')

    def try_resolve_baseline(self):
        try:
            t_left = self.tf_buffer.lookup_transform(
                self.base_frame, self.left_frame, rclpy.time.Time()
            )
            t_right = self.tf_buffer.lookup_transform(
                self.base_frame, self.right_frame, rclpy.time.Time()
            )
        except LookupException as e:
            self.get_logger().warn(
                f'Waiting for TF tree: {e}',
                throttle_duration_sec=3.0
            )
            return

        dx = t_right.transform.translation.x - t_left.transform.translation.x
        dy = t_right.transform.translation.y - t_left.transform.translation.y
        baseline_m = math.hypot(dx, dy)

        if baseline_m < 0.05:
            self.get_logger().error(
                'GPS frames are nearly co-located in URDF, check your link positions!'
            )
            return

        self.baseline_offset_rad = math.atan2(dy, dx)
        self.tf_timer.cancel()

        self.get_logger().info(
            f'Baseline resolved: {baseline_m:.3f}m at '
            f'{math.degrees(self.baseline_offset_rad):.1f}deg from robot forward'
        )

    def left_callback(self, msg):
        self.left_fix = msg
        self.try_compute_heading()

    def right_callback(self, msg):
        self.right_fix = msg
        self.try_compute_heading()

    def try_compute_heading(self):
        if self.baseline_offset_rad is None:
            return
        if self.left_fix is None or self.right_fix is None:
            return
        if self.left_fix.status.status < 0 or self.right_fix.status.status < 0:
            self.get_logger().warn(
                'No fix on one or both antennas',
                throttle_duration_sec=5.0
            )
            return

        x, y = self.latlon_to_xy(
            self.right_fix.latitude, self.right_fix.longitude,
            self.left_fix.latitude, self.left_fix.longitude
        )

        dist = math.hypot(x, y)
        if dist < self.min_baseline:
            self.get_logger().warn(
                f'Measured baseline too short ({dist:.3f}m), GPS fixes too noisy',
                throttle_duration_sec=5.0
            )
            return

        bearing_enu = math.atan2(y, x)
        robot_yaw = bearing_enu - self.baseline_offset_rad

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation = yaw_to_quaternion(robot_yaw)

        yaw_variance = (0.009 / max(dist, 0.5)) ** 2
        imu_msg.orientation_covariance[8] = yaw_variance
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.heading_pub.publish(imu_msg)

        self.get_logger().info(
            f'Heading: {math.degrees(robot_yaw):.1f}deg | baseline: {dist:.3f}m',
            throttle_duration_sec=2.0
        )

    def latlon_to_xy(self, lat, lon, origin_lat, origin_lon):
        R = 6378137.0
        dlat = math.radians(lat - origin_lat)
        dlon = math.radians(lon - origin_lon)
        mean_lat = math.radians((lat + origin_lat) / 2.0)
        x = R * dlon * math.cos(mean_lat)
        y = R * dlat
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = DualGPSHeading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()