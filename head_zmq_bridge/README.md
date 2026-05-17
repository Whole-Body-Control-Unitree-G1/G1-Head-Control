# head_zmq_bridge

Bridges ROS 2 ZED stereo camera topics to ZMQ for GR00T data collection, and streams video to a PICO VR headset for egocentric view.

## Pipeline Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                          ROBOT (G1 Orin)                            │
│                                                                     │
│  ┌──────────────┐    /zed/zed_node/stereo    ┌───────────────────┐ │
│  │  ZED Mini    │ ──/color/rect/image──────► │ head_zmq_bridge   │ │
│  │  HD720@30fps │                            │ (ROS2 node)       │ │
│  │  stereo=true │                            │                   │ │
│  └──────────────┘                            │ JPEG encode       │ │
│                                              │ msgpack pack      │ │
│                                              │ ZMQ PUB :5555     │ │
│                                              └───────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                              ZMQ tcp:5555
                                    │
                                    ▼
                       ┌────────────────────────────┐
                       │     LAPTOP                 │
                       │                            │
                       │  zed_pico_zmq.py           │
                       │  (standalone, venv_teleop) │
                       │                            │
                       │  ZMQ SUB :5555             │
                       │  H.264 encode (PyAV)       │
                       │  TCP client → PICO :12345  │
                       └──────────────┬─────────────┘
                                      │
                               TCP H.264 stream
                                      │
                                      ▼
                       ┌──────────────────────────────┐
                       │  PICO VR Headset             │
                       │  XRoboToolkit app            │
                       │                              │
                       │  displays egocentric         │
                       │  stereo view                 │
                       └──────────────────────────────┘
```

## Run Order

### 1. Robot — ZED ROS2 wrapper
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
```
Ensure `common_stereo.yaml` has:
```yaml
video:
    publish_stereo: true
```

Ensure `zedm.yaml` has:
```yaml
general:
    grab_resolution: 'HD720'
    grab_frame_rate: 30
```

### 2. Robot — ZMQ bridge
```bash
ros2 launch head_zmq_bridge zmq_bridge.launch.xml
```

### 3. Laptop — XRoboToolkit PC service
```bash
bash /opt/apps/roboticsservice/runService.sh
```
Connect the PICO to the XRoboToolkit app and ensure it is paired.

### 4. Laptop — PICO vision stream
```bash
source ~/wbcG1/GR00T-WholeBodyControl/.venv_teleop/bin/activate
python3 ~/wbcG1/headControl/src/headctrl/head_pico_bridge/head_pico_bridge/zed_pico_zmq.py
```
Then trigger the camera stream from the PICO XRoboToolkit app. The script connects to the PICO automatically on receiving the `OPEN_CAMERA` command.

## Configuration

`config/camera.yaml`:
```yaml
/**:
  ros__parameters:
    cameras: ['ego_view']
    zmq_port: 5555
    ego_view:
      topic: '/zed/zed_node/stereo/color/rect/image'
      quality: 80
```

## ZMQ Message Format

Each frame is published as a msgpack-encoded dict:
```python
{
    'timestamps': {'ego_view': float},  # ROS stamp in seconds
    'images':     {'ego_view': bytes},  # JPEG-encoded bytes
}
```
This format is compatible with GR00T's `ComposedCameraClientSensor`.

## PICO Setup

- `video_source.yml` profile: `ZEDMINI` — 2560×720, 30fps, 4Mbps H.264
- Push to PICO: `adb push video_source.yml /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml`
- PICO device ID: `PA94Y0MGK8200183G`
