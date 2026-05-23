## Head Control Package
This package is being used to control the 2-DOF head of the Unitree G1.
The head is comprised of two Dynamixel motors (Protocol 2.0):
- **Motor ID 1** — Pitch (tilt down only from neutral)
- **Motor ID 2** — Yaw (left/right)

Connected via U2D2 on `/dev/ttyUSB0` at 57600 baud.

### Prerequisites

1. Install the Dynamixel SDK via apt:
    ```
    sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk
    ```

2. Do **not** source `robotis_ws` before building — it overrides the apt installation and breaks `find_package(dynamixel_sdk)`.

### How to run

1. Build the package:
    ```
    cd ~/wbcG1/headControl
    colcon build --packages-select head_control
    source install/setup.bash
    ```

2. Calibrate (first time only):
    ```
    ros2 run head_control calibration_node
    ```
    Torque is disabled during calibration — move each joint by hand to its extremes when prompted. The node computes the center automatically and saves `calib.yaml` to the package share directory.

3. Run the control node:
    ```
    ros2 launch head_control head_control.launch.xml
    ```
    On startup the head drives to the calibrated center position automatically.

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/head/target` | `sensor_msgs/JointState` | Subscribed | Target pitch/yaw in radians |
| `/head/state` | `sensor_msgs/JointState` | Published | Current pitch/yaw in radians at 50 Hz |

### Coordinate Convention

- **Pitch**: negative = tilt down, 0 = neutral. Range: `0` to `-1.2 rad`
- **Yaw**: positive = turn right, negative = turn left. Range: `-1.57` to `+1.57 rad`

### Example Commands

Move to neutral:
```
ros2 topic pub --once /head/target sensor_msgs/msg/JointState "{name: ['pitch', 'yaw'], position: [0.0, 0.0]}"
```

Tilt down:
```
ros2 topic pub --once /head/target sensor_msgs/msg/JointState "{name: ['pitch'], position: [-0.5]}"
```

Turn right:
```
ros2 topic pub --once /head/target sensor_msgs/msg/JointState "{name: ['yaw'], position: [0.5]}"
```

### Hardware Config

| Parameter | Value |
|---|---|
| Device | `/dev/ttyUSB0` |
| Baud rate | 57600 |
| Protocol | 2.0 |
| Operating mode | Extended Position Control (mode 4) |
| Pitch motor ID | 1 |
| Yaw motor ID | 2 |
| Neutral ticks | Set by calibration (`calib.yaml`) |
