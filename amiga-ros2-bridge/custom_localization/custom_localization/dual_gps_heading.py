import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
from tf2_ros import Buffer, TransformListener, LookupException


# Max age (seconds) between left and right fix before we reject the pair
MAX_FIX_AGE_DIFF_S = 0.05


def yaw_to_quaternion(yaw_rad):
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


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

        # Angle of the right-minus-left vector in the robot's body frame (ROS: X=forward, Y=left)
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

        # --- FIX 1: reject stale fix pairs ---
        t_left = stamp_to_sec(self.left_fix.header.stamp)
        t_right = stamp_to_sec(self.right_fix.header.stamp)
        if abs(t_left - t_right) > MAX_FIX_AGE_DIFF_S:
            self.get_logger().warn(
                f'Fix timestamps differ by {abs(t_left - t_right):.3f}s, skipping',
                throttle_duration_sec=2.0
            )
            return

        if self.left_fix.status.status < 0 or self.right_fix.status.status < 0:
            self.get_logger().warn(
                'No fix on one or both antennas',
                throttle_duration_sec=5.0
            )
            return

        # right antenna minus left antenna in ENU (x=East, y=North)
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

        # Bearing of the right-minus-left vector in ENU frame.
        # atan2(y=North, x=East) gives the ENU angle (0 = East, 90 = North).
        bearing_enu = math.atan2(y, x)

        # --- FIX 2: convert ENU bearing to ROS yaw (REP-103) ---
        # ROS yaw: 0 = East (+X in ENU), 90° = North (+Y in ENU).
        # atan2(y_enu, x_enu) already IS the ROS ENU yaw, so no extra
        # rotation is needed here — the frame is the same.
        # What we DO need is to subtract the physical mounting angle
        # (baseline_offset_rad) which is expressed in the ROBOT body frame,
        # NOT in ENU. To compare, we need the body-frame bearing, which equals
        # the ENU bearing minus the robot's current yaw. Because we're solving
        # FOR the robot yaw, we rearrange:
        #
        #   bearing_body = bearing_enu - robot_yaw
        #   baseline_offset_rad = bearing_body        (definition of mounting angle)
        #   => robot_yaw = bearing_enu - baseline_offset_rad
        #
        # This is correct as long as baseline_offset_rad is measured in the
        # same sense (CCW positive) — which atan2(dy_body, dx_body) gives us.
        robot_yaw = bearing_enu - self.baseline_offset_rad

        # Normalise to (-pi, pi)
        robot_yaw = math.atan2(math.sin(robot_yaw), math.cos(robot_yaw))

        imu_msg = Imu()
        # Use the timestamp of the newer fix so downstream nodes get accurate timing
        newer_stamp = (
            self.left_fix.header.stamp
            if t_left > t_right
            else self.right_fix.header.stamp
        )
        imu_msg.header.stamp = newer_stamp
        imu_msg.header.frame_id = self.base_frame
        imu_msg.orientation = yaw_to_quaternion(robot_yaw)

        # --- FIX 3: sensible covariance ---
        # Angular uncertainty ≈ position noise / baseline length.
        # Typical RTK noise ~0.02m; use 0.03m to be conservative.
        gps_noise_m = 0.03
        yaw_std_rad = gps_noise_m / max(dist, self.min_baseline)
        imu_msg.orientation_covariance[8] = yaw_std_rad ** 2

        # Mark angular velocity and linear acceleration as not provided
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.heading_pub.publish(imu_msg)

        self.get_logger().info(
            f'Heading: {math.degrees(robot_yaw):.1f}deg (ENU yaw) | baseline: {dist:.3f}m',
            throttle_duration_sec=2.0
        )

    def latlon_to_xy(self, lat, lon, origin_lat, origin_lon):
        """Returns (x=East, y=North) displacement in metres from origin to (lat,lon)."""
        R = 6378137.0
        dlat = math.radians(lat - origin_lat)
        dlon = math.radians(lon - origin_lon)
        mean_lat = math.radians((lat + origin_lat) / 2.0)
        x = R * dlon * math.cos(mean_lat)   # East
        y = R * dlat                          # North
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = DualGPSHeading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()