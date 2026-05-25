import json
import math
import numpy as np
from scipy.spatial.transform import Rotation as sRot
import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ZMQ_PORT  = 5556
ZMQ_TOPIC = "pose"

ZMQ_STATE_PORT  = 5558
ZMQ_STATE_TOPIC = "head_state"

# vr_orientation is float[12]: [left_wxyz, right_wxyz, head_wxyz]
HEAD_QUAT_OFFSET = 8

DTYPE_MAP = {
    "f32": np.float32,
    "f64": np.float64,
    "i32": np.int32,
    "i64": np.int64,
    "bool": np.bool_,
}


HEADER_SIZE = 1280

def unpack_pose_message(raw: bytes, topic: str) -> dict:
    payload = raw[len(topic):]
    header = json.loads(payload[:HEADER_SIZE].rstrip(b"\x00"))
    binary = payload[HEADER_SIZE:]

    arrays = {}
    offset = 0
    for field in header["fields"]:
        dtype = DTYPE_MAP[field["dtype"]]
        shape = field["shape"]
        nbytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
        arrays[field["name"]] = np.frombuffer(
            binary[offset : offset + nbytes], dtype=dtype
        ).reshape(shape)
        offset += nbytes
    return arrays


def pack_state_message(pitch: float, yaw: float) -> bytes:
    data = np.array([pitch, yaw], dtype=np.float32)
    fields = [{"name": "head_state", "dtype": "f32", "shape": [2]}]
    header = json.dumps({"fields": fields, "version": 1, "count": 1}).encode().ljust(1024, b"\x00")
    return ZMQ_STATE_TOPIC.encode() + header + data.tobytes()


class HeadPicoBridge(Node):
    def __init__(self):
        super().__init__("head_pico_bridge")

        self.declare_parameter("zmq_host", "192.168.36.214")
        zmq_host = self.get_parameter("zmq_host").get_parameter_value().string_value

        self._pub = self.create_publisher(JointState, "head/target", 10)

        ctx = zmq.Context()

        self._sock = ctx.socket(zmq.SUB)
        self._sock.connect(f"tcp://{zmq_host}:{ZMQ_PORT}")
        self._sock.setsockopt_string(zmq.SUBSCRIBE, ZMQ_TOPIC)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "planner")

        self._state_sock = ctx.socket(zmq.PUB)
        self._state_sock.bind(f"tcp://*:{ZMQ_STATE_PORT}")

        self._state_sub = self.create_subscription(
            JointState, "head/state", self._state_callback, 10
        )

        self._calibration_ref = None
        self._prev_a = False

        self.create_timer(0.02, self._poll)  # 50 Hz
        self.get_logger().info(
            f"Head PICO bridge started — "
            f"SUB tcp://{zmq_host}:{ZMQ_PORT}, "
            f"PUB tcp://*:{ZMQ_STATE_PORT}"
        )
        self.get_logger().info("Head will auto-calibrate on first message. Press right menu button to re-calibrate.")

    def _poll(self):
        try:
            raw = self._sock.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        topic = "planner" if raw.startswith(b"planner") else ZMQ_TOPIC
        try:
            arrays = unpack_pose_message(raw, topic)
        except Exception as e:
            self.get_logger().warn(f"Failed to unpack message: {e}")
            return

        ori = arrays.get("vr_orientation")
        if ori is None or ori.size < 12:
            self.get_logger().warn("vr_orientation missing or too short")
            return

        ori = ori.flatten()
        head_q = ori[HEAD_QUAT_OFFSET:HEAD_QUAT_OFFSET + 4]  # [w, x, y, z]

        # head_q is [w,x,y,z]; scipy from_quat expects [x,y,z,w]
        head_q_xyzw = [head_q[1], head_q[2], head_q[3], head_q[0]]

        # Auto-calibrate on first message ever received
        if self._calibration_ref is None:
            self._calibration_ref = sRot.from_quat(head_q_xyzw)
            self.get_logger().info("Head zero pose auto-calibrated on first message")

        # Right menu button rising edge — re-calibrate manually (POSE mode only)
        right_menu = arrays.get("right_menu_button")
        btn_pressed = bool(right_menu[0]) if right_menu is not None else False
        if btn_pressed and not self._prev_a:
            self._calibration_ref = sRot.from_quat(head_q_xyzw)
            self.get_logger().info("Head zero pose re-calibrated")
        self._prev_a = btn_pressed

        # Apply calibration; result is [x,y,z,w]
        q_cal = (self._calibration_ref.inv() * sRot.from_quat(head_q_xyzw)).as_quat()
        x, y, z, w = float(q_cal[0]), float(q_cal[1]), float(q_cal[2]), float(q_cal[3])

        pitch, yaw = quat_to_pitch_yaw(w, x, y, z)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ["pitch", "yaw"]
        msg.position = [pitch, yaw]
        self._pub.publish(msg)

    def _state_callback(self, msg: JointState):
        pitch, yaw = 0.0, 0.0
        for i, name in enumerate(msg.name):
            if name == "pitch" and i < len(msg.position):
                pitch = msg.position[i]
            elif name == "yaw" and i < len(msg.position):
                yaw = msg.position[i]
        self._state_sock.send(pack_state_message(pitch, yaw))


def quat_to_pitch_yaw(w, x, y, z):
    # PICO frame: X=roll, Y=nod, Z=look
    pitch = -math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))           # Y rot = nod (negated for robot convention)
    yaw   = -math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) # Z rot = look (negated for mirrored orientation)
    return pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = HeadPicoBridge()
    rclpy.spin(node)
    rclpy.shutdown()
