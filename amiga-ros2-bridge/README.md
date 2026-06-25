# amiga-ros2-bridge (AEWD Copy)

The project in this folder is a copy of the following ROS2 bridge project:
[ucmercedrobotics/amiga-ros2-bridge](https://github.com/ucmercedrobotics/amiga-ros2-bridge.git)

We do not own that project; all credits go to the original creators.
Development of the AEWD project was done in this copy for ease of development.

---

## History

- We got the `amiga-ros2-bridge` to build.
- We ran some experiments toward the goal of autonomous **agricultural** navigation.
- We lobotomized the project — removing everything we did not need and keeping only what was necessary.
- We continued development here because it was easiest, telling ourselves we would eventually extract our code into our own Docker container and build the `amiga-ros2-bridge` separately. **We did not achieve that goal.**

---

## What We Added / Changed

- Customized `amiga_ros2_description` to display our hardware in the URDF.
- Created `custom_localization`
- Created `custom_navigation`

---

## Recommended Next Steps for the architecture

### Eighter Dual Docker Setup
1. Extract the modified `amiga_ros2_description`, `custom_localization`, and `custom_navigation` folders.
2. Place these folders in a similar Docker container.
3. Delete this copy of the `amiga-ros2-bridge`.
4. Spin up a fresh, up-to-date, unmodified Docker image of the `amiga-ros2-bridge`.
5. Build both projects side by side:
   - Launch `amiga_ros2_description`, `custom_localization`, and `custom_navigation` in the new AEWD image.
   - Launch `amiga-streams` and `twist` in the new `amiga-ros2-bridge` image.
   - Have both projects communicate with each other via ROS2 network functionality.

### Or native ROS2 on Ubuntu *(better but a lot of work)*
Extract the modified `amiga_ros2_description`, `custom_localization`, and `custom_navigation` folders.
Run ROS2 Humble natively on an Ubuntu device, add these folders to a ROS2 workspace, and write a ROS2 node that translates `Twist` messages to CAN bus messages.
Then directly connect the device to the Amiga robot's CAN bus.

A completely local device would circumvent the numerous network errors, random disconnects, and long, tedious build times.

> These options are a significant amount of work and should **not** be treated as a high priority.