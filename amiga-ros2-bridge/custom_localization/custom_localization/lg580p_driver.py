#!/usr/bin/env python3

import serial
import math

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node


class LG580PDriver(Node):

    def __init__(self):

        super().__init__("lg580p_driver")

        #
        # Parameters
        #
        self.latitude = 0.0
        self.longitude = 0.0

        self.declare_parameter(
            "port",
            "/dev/ttyACM0"
        )

        self.declare_parameter(
            "baud",
            460800
        )

        self.declare_parameter(
            "gps_frame",
            "lg580p_link"
        )

        self.port = self.get_parameter(
            "port"
        ).value

        self.baud = self.get_parameter(
            "baud"
        ).value

        self.frame = self.get_parameter(
            "gps_frame"
        ).value

        #
        # GGA data
        #
        self.fix_quality = -1
        self.satellites = 0
        self.hdop = 0.0
        self.altitude = 0.0

        #
        # TAR data
        #
        self.tar_quality = -1
        self.heading_deg = 0.0
        self.pitch_deg = 0.0
        self.baseline_m = 0.0
        self.utc_time = "UNKNOWN"

        #
        # Counters
        #
        self.total_count = 0
        self.gga_count = 0
        self.tar_count = 0

        #
        # Startup logging
        #
        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "LG580P Driver Starting"
        )

        self.get_logger().info(
            f"Port      : {self.port}"
        )

        self.get_logger().info(
            f"Baud      : {self.baud}"
        )

        self.get_logger().info(
            f"Frame     : {self.frame}"
        )

        #
        # Serial
        #
        self.serial = serial.Serial(
            self.port,
            self.baud,
            timeout=1.0
        )

        self.get_logger().info(
            f"Serial open: {self.serial.is_open}"
        )

        self.get_logger().info(
            "================================="
        )

        #
        # Timers
        #
        self.read_timer = self.create_timer(
            0.01,
            self.read_serial
        )

        self.publish_timer = self.create_timer(
            0.1,
            self.publish_ros_messages
        )

        self.status_timer = self.create_timer(
            1.0,
            self.print_status
        )

        self.fix_pub = self.create_publisher(
            NavSatFix,
            "/gnss/fix",
            10
        )

        self.heading_pub = self.create_publisher(
            Imu,
            "/gnss/heading",
            10
        )

    def nmea_to_decimal(self, value, direction):

        if not value:
            return 0.0

        raw = float(value)

        degrees = int(raw / 100)
        minutes = raw - (degrees * 100)

        decimal = degrees + minutes / 60.0

        if direction in ["S", "W"]:
            decimal *= -1.0

        return decimal


    def parse_gga(self, line):

        fields = line.split(",")

        if len(fields) < 10:
            return

        try:

            self.latitude = self.nmea_to_decimal(
                fields[2],
                fields[3]
            )

            self.longitude = self.nmea_to_decimal(
                fields[4],
                fields[5]
            )

            self.fix_quality = int(fields[6])

            self.satellites = int(fields[7])

            self.hdop = float(fields[8])

            self.altitude = float(fields[9])

        except Exception as e:

            self.get_logger().warn(
                f"GGA parse failed: {e}"
            )
    #
    # PQTMTAR
    #
    def parse_tar(self, line):

        try:

            fields = line.split(",")

            if len(fields) < 13:
                return

            self.utc_time = fields[2]

            self.tar_quality = int(fields[3])

            self.baseline_m = float(fields[5])

            self.pitch_deg = float(fields[6])

            self.heading_deg = float(fields[8])

            self.acc_pitch_deg = float(fields[9])

            self.acc_heading_deg = float(
                fields[11].split("*")[0]
            )

            self.used_sv = int(
                fields[12].split("*")[0]
            )

        except Exception as e:

            self.get_logger().warn(
                f"TAR parse failed: {e}"
            )

            self.get_logger().warn(
                f"RAW TAR: {line}"
            )

    #
    # Serial
    #
    def read_serial(self):

        try:

            line = (
                self.serial.readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
            )

            if not line:
                return

            self.total_count += 1

            if line.startswith("$GNGGA"):

                self.gga_count += 1

                self.parse_gga(
                    line
                )

            elif line.startswith("$PQTMTAR"):

                self.tar_count += 1

                self.parse_tar(
                    line
                )

        except Exception as e:

            self.get_logger().error(
                f"Serial error: {e}"
            )

    #
    # RTK status
    #
    def tar_quality_string(self):

        if self.tar_quality == 4:
            return "RTK FIXED"

        elif self.tar_quality == 5:
            return "RTK FLOAT"

        elif self.tar_quality == 2:
            return "DGPS"

        elif self.tar_quality == 1:
            return "GNSS"

        return "INVALID"

    def heading_valid(self):

        return (
            self.tar_quality == 4
        )
    
    def publish_ros_messages(self):

        #
        # NavSatFix
        #
        fix = NavSatFix()

        fix.header.stamp = (
            self.get_clock().now().to_msg()
        )

        fix.header.frame_id = self.frame

        fix.status.status = (
            NavSatStatus.STATUS_FIX
        )

        fix.status.service = (
            NavSatStatus.SERVICE_GPS
        )

        fix.latitude = self.latitude
        fix.longitude = self.longitude
        fix.altitude = self.altitude

        gps_sigma = max(
            self.hdop * 0.5,
            0.3
        )

        gps_var = gps_sigma * gps_sigma

        fix.position_covariance = [
            gps_var, 0.0,     0.0,
            0.0,     gps_var, 0.0,
            0.0,     0.0,     25.0
        ]

        fix.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        )

        self.fix_pub.publish(
            fix
        )

        #
        # Heading IMU
        #
        imu = Imu()

        imu.header.stamp = (
            fix.header.stamp
        )

        imu.header.frame_id = self.frame

        #
        # GNSS heading:
        #
        # 0 deg = North
        # 90 deg = East
        #
        yaw_deg = (
            90.0 -
            self.heading_deg
        )

        yaw_rad = math.radians(
            yaw_deg
        )

        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(
            yaw_rad / 2.0
        )
        imu.orientation.w = math.cos(
            yaw_rad / 2.0
        )

        #
        # Ignore roll/pitch
        #
        imu.orientation_covariance = [
            99999.0, 0.0,     0.0,
            0.0,     99999.0, 0.0,
            0.0,     0.0,     0.05
        ]

        self.heading_pub.publish(
            imu
        )

    #
    # Console status
    #
    def print_status(self):

        #
        # Quality according to Quectel/Sparkfun docs
        #
        quality_map = {
            0: "INVALID",
            1: "GPS SPS",
            2: "DGPS/SBAS",
            3: "GPS PPS",
            4: "FIX HEADING",
            5: "FLOAT HEADING",
        }

        quality_str = quality_map.get(
            self.tar_quality,
            f"UNKNOWN({self.tar_quality})"
        )

        self.get_logger().info(
            "\n"
            "=====================================================\n"
            "LG580P PQTMTAR CHECK\n"
            "=====================================================\n"
            f"UTC Time            : {self.utc_time}\n"
            f"Heading Quality     : {self.tar_quality} ({quality_str})\n"
            f"Heading             : {self.heading_deg:.3f} deg\n"
            f"Heading Accuracy    : {self.acc_heading_deg:.3f}\n"
            f"Pitch               : {self.pitch_deg:.3f} deg\n"
            f"Pitch Accuracy      : {self.acc_pitch_deg:.3f}\n"
            f"Baseline Length     : {self.baseline_m:.3f} m\n"
            f"Used Satellites     : {self.used_sv}\n"
            "\n"
            f"Latitude            : {self.latitude:.8f}\n"
            f"Longitude           : {self.longitude:.8f}\n"
            f"GGA Fix Quality     : {self.fix_quality}\n"
            f"HDOP                : {self.hdop:.2f}\n"
            f"Altitude            : {self.altitude:.3f} m\n"
            "\n"
            f"Messages            : {self.total_count}\n"
            f"GGA                 : {self.gga_count}\n"
            f"PQTMTAR             : {self.tar_count}\n"
            "====================================================="
        )


def main(args=None):

    rclpy.init(args=args)

    node = LG580PDriver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()