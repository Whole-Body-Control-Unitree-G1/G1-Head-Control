# head_pico_bridge

Bridges PICO headset orientation data into ROS 2.

## Overview

Two components work together:

- **`xrt_head_publisher.py`** — standalone Python script that reads body tracking data from the XRobotToolkit SDK, computes the head orientation relative to the root body joint, and publishes it via ZMQ PUB on port `5556`.
- **`bridge_node.py`** — ROS 2 node that subscribes to the ZMQ stream and publishes the head pitch/yaw as a `sensor_msgs/JointState` on the `head/target` topic.

## Architecture

```
PICO Headset
    |
xrt_head_publisher.py  (ZMQ PUB, port 5556, topic "pose")
    |
bridge_node (ROS 2)
    |
/head/target  [sensor_msgs/JointState]
```

## Dependencies

- `xrobotoolkit_sdk` — must be installed on the machine running `xrt_head_publisher.py`
- `rclpy`, `sensor_msgs`
- `pyzmq`, `numpy`, `scipy`

## Usage

**1. Start the XRT publisher** (on the machine with the PICO headset):

```bash
python3 xrt_head_publisher.py
```

Press **A** on the headset to calibrate the zero pose.

**2. Launch the ROS 2 bridge node:**

```bash
ros2 run head_pico_bridge head_pico_bridge
```

## Calibration

On startup, `xrt_head_publisher.py` waits for a calibration step. Press the **A button** on the PICO controller to set the current head pose as the zero reference. All subsequent orientations are relative to this pose.
