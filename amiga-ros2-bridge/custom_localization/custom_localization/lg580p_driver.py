#!/usr/bin/env python3

import serial
import math

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node

# This node reads the lg580p module serial bus (attached via usb). Publishes both gnss/fix (long, lat) and gnss/heading (Imu topic with only yaw).
# Furthermore it logs at 1hz the data coming from the lg580p for debugging purposes.

# This node does two things at the same time. That defies the single responsibility guideline sadly. But its for a reason.
# This node is the only node that reads the serial bus to the lg580p module. And having multiple nodes open that same serial bus at the same time leads to problems.
# So for convenience we left all the code here in one big node instead of making a proper segmented read-serial, publish-heading, publish-fix classes.

# The documentation for the lg580p can be found across these websites:
# https://docs.sparkfun.com/SparkFun_GNSS_LG580P/pqmt_commands/#heading-settings
# https://docs.sparkfun.com/SparkFun_GNSS_LG580P/hardware_assembly/
# https://docs.sparkfun.com/SparkFun_GNSS_LG580P/assets/component_documentation/quectel_lg580p_series_dual-antenna_heading_application_note_v1-0.pdf

# The top website pqmt_commands/#heading-settings is by far the most useful for this code block. And was the primary source of truth.
# The LG580p gets read out via NMEA. These 2 sentences are most important:
# - Heading (referred to as TAR) = $PQTMTAR,<MsgVer>,<Time>,<Quality>,<Res>,<Length>,<Pitch>,<Roll>,<Heading>,<Acc_Pitch>,<Acc_Roll>,<Acc_Heading>,<UsedSV>*<Checksum><CR><LF>
# - Fix (referred to as GGA)     = $GNGGA,<UTC>,<Lat>,<N/S>,<Long>,<E/W>,<Quality>,<NumSV>,<HDOP>,<Alt>,M,<GeoSep>,M,<DiffAge>,<DiffRef>*<Checksum><CR><LF>

# Suggested improvement:
# - make heading use acc_heading_deg to set the orientation_covariance to real numbers rather than leaving it at 99999.

class LG580PDriver(Node):

    def __init__(self):

        super().__init__("lg580p_driver")

        # Parameters
        # port, baud, gps_frame, antenna_offset_deg are set by the launch file.
        # But they are also declared here with defaults to prevent crashes.
        self.latitude = 0.0
        self.longitude = 0.0

        # also set by launch file
        self.declare_parameter(
            "port",
            "/dev/ttyACM0"
        )

        # also set by launch file
        self.declare_parameter(
            "baud",
            460800
        )

        # also set by launch file
        self.declare_parameter(
            "gps_frame",
            "lg580p_link"
        )

        # also set by launch file
        self.declare_parameter(
            "antenna_offset_deg",
            90.0  # default: antennas side-by-side (West=0 convention)
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

        self.antenna_offset_deg = self.get_parameter(
            "antenna_offset_deg"
        ).value

        # GGA data
        self.fix_quality = -1
        self.satellites = 0
        self.hdop = 0.0
        self.altitude = 0.0

        # TAR data
        self.tar_quality = -1
        self.heading_deg = 0.0
        self.pitch_deg = 0.0
        self.baseline_m = 0.0
        self.utc_time = "UNKNOWN"

        # Counters
        self.total_count = 0
        self.gga_count = 0
        self.tar_count = 0


        # Startup logging
        self.get_logger().info("=================================")
        self.get_logger().info("LG580P Driver Starting")
        self.get_logger().info(f"Port      : {self.port}")
        self.get_logger().info(f"Baud      : {self.baud}")
        self.get_logger().info(f"Frame     : {self.frame}")
        self.get_logger().info(f"Antenna offset: {self.antenna_offset_deg} deg")
        # Serial
        self.serial = serial.Serial(
            self.port,
            self.baud,
            timeout=1.0
        )
        self.get_logger().info(f"Serial open: {self.serial.is_open}")
        self.get_logger().info("=================================")


        # Timers
        self.read_timer = self.create_timer(
            0.01,
            self.read_serial
        )

        self.publish_timer = self.create_timer(
            0.05,
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

    # NMEA encodes coordinates as DDDMM.MMMMM instead of decimal degrees. A quirk of the format.
    # So 5230.50000,N means 52 degrees + 30.5 minutes North. This converts that to plain decimal (52.5083...).
    # The direction field (N/S/E/W) handles the sign. South and West come out negative.
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

    # Pulls position data out of the GGA sentence. Standard NMEA. All GPS modules speak this.
    # Fields we pull:
    # $GNGGA, <UTC>, <Lat>, <N/S>, <Long>, <E/W>, <Quality>, <NumSV>, <HDOP>, <Alt>,M,<GeoSep>,M,<DiffAge>,<DiffRef>*<Checksum><CR><LF>
    #                 [2]    [3]    [4]     [5]     [6]        [7]      [8]     [9]
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

    # This is Sparkfuns proprietary dual-antenna heading sentence. (the reason why this node is not a premade but custom node)
    # Fields we pull:
    # $PQTMTAR, <MsgVer>, <Time>, <Quality>, <Res>, <Length>, <Pitch>, <Roll>, <Heading>, <Acc_Pitch>, <Acc_Roll>, <Acc_Heading>, <UsedSV>*<Checksum><CR><LF>
    #                      [2]     [3]                [5]       [6]              [8]         [9]                      [11]            [12]
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

    # Reads one line from the serial port and routes it to the right parser based on the sentence prefix. You can do cat /dev/ttyACM0 in terminal to see the raw data.
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

    # RTK status taken from the docs
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

    # Currently used to remove all inaccurate heading readings from being published.
    def heading_valid(self):
        return (
            self.tar_quality == 4
        )
    
    def publish_ros_messages(self):
        # NavSatFix
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

        # Heading IMU
        imu = Imu()

        imu.header.stamp = (
            fix.header.stamp
        )

        imu.header.frame_id = self.frame

        # GNSS heading conversion:
        # Chip convention  : antenna_offset_deg = 0 -> West=0, clockwise increase
        #                    antenna_offset_deg = 90 -> North=0, clockwise increase (standard NMEA)
        # ROS/ENU convention: East=0, counter-clockwise increase
        yaw_deg = (
            (90.0 - (self.heading_deg - self.antenna_offset_deg)) % 360.0
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

        # Ignore roll/pitch
        imu.orientation_covariance = [
            99999.0, 0.0,     0.0,
            0.0,     99999.0, 0.0,
            0.0,     0.0,     0.05
        ]

        if not self.heading_valid():
            return

        self.heading_pub.publish(imu)


    # Console status
    def print_status(self):
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
            f"Heading (chip)      : {self.heading_deg:.3f} deg\n"
            f"Heading (published) : {((90.0 - (self.heading_deg - self.antenna_offset_deg))%360):.3f} deg (ENU)\n"
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