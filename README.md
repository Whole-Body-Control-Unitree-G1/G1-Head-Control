# G1 Head Control

**Author:** Jyothiswaroop Kasina

ROS 2 package for the 2-DOF head of the Unitree G1 robot.

[](https://github.com/user-attachments/assets/dfd70f2d-9fb1-436c-9037-9ec9a890800a)

## Packages

| Package | Description |
|---|---|
| [`head_control`](head_control/) | ROS 2 node to drive the two Dynamixel motors (pitch and yaw) via Dynamixel SDK. Subscribes to `/head/target` and publishes current joint state on `/head/state`. |
| [`head_pico_bridge`](head_pico_bridge/) | Bridges PICO headset orientation data to ROS 2. Reads head pose from the XRobotToolkit SDK, publishes via ZMQ, and a ROS 2 node converts it to `JointState` on `/head/target`. |
| [`head_zmq_bridge`](head_zmq_bridge/) | Bridges ROS 2 camera topics to ZMQ for GR00T data collection. Creates one ZMQ PUB socket per camera, publishing msgpack-encoded JPEG frames. |

## Getting Started

After cloning, build the knowledge graph for AI-assisted development:

```bash
/graphify .
```

This generates `graphify-out/` locally (not tracked in git) and enables Claude Code to answer architecture questions about the codebase. The graph rebuilds automatically on every `git commit` from that point on.

## Neck Design

The mechanical neck design is based on the [TWIST2](https://yanjieze.com/TWIST2/) project.
