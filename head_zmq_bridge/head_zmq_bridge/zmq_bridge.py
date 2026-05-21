import cv2
import msgpack
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq


class ZMQBridgeNode(Node):

    def __init__(self):
        super().__init__('zmq_bridge')
        self.get_logger().info('zmq_bridge_node')

        self.create_config()
        self.bridge = CvBridge()

        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind(f'tcp://*:{self.zmq_port}')
        self.standalone_socket = self.ctx.socket(zmq.PUB)
        self.standalone_socket.setsockopt(zmq.CONFLATE, 1)
        self.standalone_socket.bind(f'tcp://*:{self.standalone_zmq_port}')

        # Synchronized subscribers for GR00T cameras (combined message, time-aligned)
        if self.sync_cameras:
            sync_subs = [
                Subscriber(self, Image, self.camera_config[n]['topic'])
                for n in self.sync_cameras
            ]
            self.sync_qualities = [self.camera_config[n]['quality'] for n in self.sync_cameras]
            ts = ApproximateTimeSynchronizer(sync_subs, queue_size=10, slop=0.05)
            ts.registerCallback(self.sync_callback)

        # Standalone subscribers (one message per camera, e.g. stereo for PICO)
        self.standalone_subs = {}
        for name in self.standalone_cameras:
            self.standalone_subs[name] = self.create_subscription(
                Image,
                self.camera_config[name]['topic'],
                lambda msg, n=name: self.standalone_callback(msg, n),
                10,
            )

    def create_config(self):
        self.declare_parameter('sync_cameras', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('standalone_cameras', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('zmq_port', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('standalone_zmq_port', rclpy.Parameter.Type.INTEGER)

        self.sync_cameras = self.get_parameter('sync_cameras').get_parameter_value().string_array_value
        self.standalone_cameras = self.get_parameter('standalone_cameras').get_parameter_value().string_array_value
        self.zmq_port = self.get_parameter('zmq_port').value
        self.standalone_zmq_port = self.get_parameter('standalone_zmq_port').value

        self.camera_config = {}
        for name in list(self.sync_cameras) + list(self.standalone_cameras):
            self.declare_parameter(f'{name}.topic', rclpy.Parameter.Type.STRING)
            self.declare_parameter(f'{name}.quality', rclpy.Parameter.Type.INTEGER)
            self.declare_parameter(f'{name}.width', value=0)
            self.declare_parameter(f'{name}.height', value=0)
            w = self.get_parameter(f'{name}.width').value
            h = self.get_parameter(f'{name}.height').value
            self.camera_config[name] = {
                'topic':       self.get_parameter(f'{name}.topic').get_parameter_value().string_value,
                'quality':     self.get_parameter(f'{name}.quality').value,
                'target_size': (w, h) if w > 0 and h > 0 else None,
            }

    def _encode(self, msg, quality, target_size=None):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if target_size is not None:
            frame = cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)
        _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return ts, jpg.tobytes()

    def sync_callback(self, *msgs):
        """Fires when all GR00T cameras have a time-aligned frame."""
        timestamps = {}
        images = {}
        for name, msg in zip(self.sync_cameras, msgs):
            ts, jpg = self._encode(msg, self.camera_config[name]['quality'], self.camera_config[name]['target_size'])
            timestamps[name] = ts
            images[name] = jpg
        self.socket.send(msgpack.packb(
            {'timestamps': timestamps, 'images': images},
            use_bin_type=True,
        ))

    def standalone_callback(self, msg, name):
        """Fires independently for each standalone camera (e.g. stereo for PICO)."""
        ts, jpg = self._encode(msg, self.camera_config[name]['quality'], self.camera_config[name]['target_size'])
        self.standalone_socket.send(msgpack.packb(
            {'timestamps': {name: ts}, 'images': {name: jpg}},
            use_bin_type=True,
        ))


def main(args=None):
    rclpy.init(args=args)
    node = ZMQBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
