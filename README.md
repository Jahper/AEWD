# AEWD — Autonomous Electric Tool Carrier

**AEWD** (Autonome Elektrische Werktuig Drager) is a robot designed to drive autonomously within strip cropping (strokenteelt) agricultural environments.

This repository contains a proof-of-concept navigation stack based on the [Amiga ROS Bridge](https://github.com/ucmercedrobotics/amiga-ros2-bridge). The Amiga platform was chosen because all prototyping took place on it.

The repository includes a working `Dockerfile` to set up the bridge without build errors and with all dependencies required to run Nav2. In the future, replacing the bridge entirely would be a worthwhile improvement — currently only the Amiga streams and `Twist` are used to forward `cmd_vel` commands from Nav2 to the Amiga.

---

## Table of Contents

- [Hardware List](#hardware-list)
- [GPS Setup](#gps-setup)
- [IMU Setup](#imu-setup)
- [Getting Started](#getting-started)
  - [Step 1 — Install Dependencies](#step-1--install-dependencies)
  - [Step 2 — Clone the Repository](#step-2--clone-the-repository)
  - [Step 3 — Build the Docker Image](#step-3--build-the-docker-image)
  - [Step 4 — Open the First Terminal](#step-4--open-the-first-terminal)
  - [Step 5 — Open Additional Terminals](#step-5--open-additional-terminals)
  - [Step 6 — Connect to the Amiga](#step-6--connect-to-the-amiga)
  - [Step 7 — Build the Workspace](#step-7--build-the-workspace)
  - [Step 8 — Start Amiga Streams](#step-8--start-amiga-streams)
  - [Step 9 — Start Twist](#step-9--start-twist)
  - [Step 10 — Launch Custom Localisation](#step-10--launch-custom-localisation)
  - [Step 11 — Launch Nav2](#step-11--launch-nav2)
  - [Step 12 — Start RViz2](#step-12--start-rviz2)

---

## Hardware List

| Component | Description |
|-----------|-------------|
| Amiga | Main robot platform |
| 2× Amiga Intelligence Kit / u-blox GPS Antenna | GPS antennas mounted on the robot (left = primary, right = heading) |
| LG580P GPS chip | Processes GPS signals and determines heading for odometry |
| ZED-F9P chip + u-blox antenna | RTK base station for centimetre-level positioning accuracy |
| BNO055 IMU | Inertial Measurement Unit for orientation and motion data |
| ESP32 | Microcontroller used to read and forward IMU data |
| Jetson / Laptop | Runs the ROS2 bridge and navigation stack |

---

## GPS Setup

Two Intelligence Kits are mounted on the Amiga and used as GPS antennas. These antennas connect to an **LG580P GPS chip**, which also determines the heading used for odometry.

**Important notes:**
- The **left antenna** must be configured as the **primary antenna**, plugged into slot 1.
- The **right antenna** is used primarily for heading determination, pligged into slot 2.
- The system uses **GPS RTK**, so setting up a reliable base station is essential.
- If the Intelligence Kits are relocated on the robot, the [robot description](https://github.com/Jahper/AEWD/blob/main/amiga-ros2-bridge/amiga_ros2_description/urdf/amiga_descr.urdf.xacro) **must be updated** to reflect the new positions.

---

## IMU Setup

The IMU firmware can be found [here](https://github.com/Jahper/AEWD/tree/main/custom_imu_esp32) and must be flashed onto an ESP32 using **PlatformIO**. Connect the IMU using the default **I2C pins**.

**Important notes:**
- The IMU is **optional** — the Amiga can operate without it.
- When GPS accuracy is low, the IMU provides corrections to maintain reliable localisation.

---

## Getting Started

All steps below must be followed **in order**.

### Step 1 — Install Dependencies

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install git
sudo apt install python3
sudo apt install docker.io
```

### Step 2 — Clone the Repository

```bash
git clone https://github.com/Jahper/AEWD.git
cd AEWD/amiga-ros2-bridge
```

### Step 3 — Build the Docker Image

```bash
sudo make build-image udev
```

### Step 4 — Open the First Terminal and build MicroRos Agent

```bash
sudo make bash

source /opt/ros/$ROS_DISTRO/setup.bash

mkdir imu_agent && cd imu_agent

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

### Step 5 — Open Additional Terminals

For each additional terminal needed, run:

```bash
cd AEWD/amiga-ros2-bridge
sudo make shell
```

### Step 6 — Connect to the Amiga

Connect to the Amiga via **Wi-Fi** or **Ethernet**. The Ethernet connection is recommended for better performance and reliability.

#### Connecting via Ethernet (Ubuntu)

**1. Connect the Ethernet cable**

Plug the Ethernet cable directly between your Ubuntu laptop and the robot's Ethernet port.

**2. Find the robot's IP address**

On the robot's debug terminal:

```bash
ip addr show eth0
```

Look for the `inet` line, for example:

```
inet 10.95.76.1/24 scope global eth0
```

- Robot IP: `10.95.76.1`
- Subnet: `/24`

**3. Configure the laptop's Ethernet interface**

First, find your laptop's Ethernet interface name:

```bash
ip addr
```

Common interface names: `eth0`, `enp0s31f6`, `eno1`

Then assign an IP address in the same subnet:

```bash
sudo ip addr flush dev enp0s31f6
sudo ip addr add 10.95.76.2/24 dev enp0s31f6
sudo ip link set enp0s31f6 up
```

> Replace `enp0s31f6` with your actual interface name, and `10.95.76.2` with any free IP in the same subnet.

**4. Test the connection**

```bash
ping 10.95.76.1
```

If you receive replies, the Ethernet connection is working.

**5. Update the bridge configuration**

In `service_config.json`, set all `host` fields to the robot's IP address:

```json
"host": "10.95.76.1"
```

This setup provides stable ROS2 bridge communication, reliable CAN command delivery, and better performance than Wi-Fi or hotspot.

#### Useful Debugging Commands

```bash
# Show laptop network interfaces
ip addr

# Check robot Ethernet interface
ip addr show eth0

# Test if the bridge port is reachable
nc -vz 10.95.76.1 6001

# Show listening services on the robot
ss -tulpn
```

---

### Step 7 — Build the Workspace

```bash
colcon build
source install/setup.bash
make description
```

### Step 8 — Start Amiga Streams

```bash
make amiga-streams
```

### Step 9 — Start Twist

```bash
make twist
```

### Step 10 — Launch Custom Localisation

```bash
ros2 launch custom_localization gps.launch.py
```

### Step 11 — Launch Nav2

```bash
ros2 launch custom_navigation nav2.launch.py
```

### Step 12 — Start RViz2

```bash
rviz2
```