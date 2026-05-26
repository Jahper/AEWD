##################################################
## TEMPORARY SCRIPT FOR ONLY GPS BASED ODOMETRY ##
##################################################

# gpt generated.
# for propper odometry we need to introduce an imu or a second gps. then via navsat and ekf nodes create a propper odom frame.
# a propper map manager would also help because as of now we are not using map. just setting the first recieved gps location as (0,0) 

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class RTKLocalizer(Node):

    def __init__(self):
        super().__init__('rtk_localizer')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odometry/gps',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.get_logger().info('RTK Localizer Started')

    def gps_callback(self, msg):

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        # Store first GPS fix as local origin
        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt

            self.get_logger().info(
                f'Set GPS origin: lat={lat}, lon={lon}, alt={alt}'
            )

        # Convert lat/lon to local meters
        x, y = self.latlon_to_xy(
            lat,
            lon,
            self.origin_lat,
            self.origin_lon
        )

        # Publish odometry
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Identity orientation
        odom.pose.pose.orientation.w = 1.0

        self.odom_publisher.publish(odom)

        # Publish TF
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def latlon_to_xy(self, lat, lon, origin_lat, origin_lon):

        # Earth's radius in meters
        R = 6378137.0

        dlat = math.radians(lat - origin_lat)
        dlon = math.radians(lon - origin_lon)

        mean_lat = math.radians((lat + origin_lat) / 2.0)

        x = R * dlon * math.cos(mean_lat)
        y = R * dlat

        return x, y


def main(args=None):

    rclpy.init(args=args)

    node = RTKLocalizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()