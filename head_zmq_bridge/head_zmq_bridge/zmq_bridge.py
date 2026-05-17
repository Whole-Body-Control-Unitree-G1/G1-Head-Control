import cv2
import msgpack
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq


class ZMQBridgeNode(Node):
    """ZMQBridgeNode class."""
    def __init__(self):
        """
        Create ZMQBridgeNode.

        Reads camera config from ROS parameters and creates one
        subscriber and one ZMQ PUB socket per camera.

        Subscribers
        -----------
        <camera>.topic : sensor_msgs/msg/Image
            One subscriber per camera, topic defined in config.

        ZMQ Publishers
        --------------
        tcp://*:<camera>.zmq_port : msgpack
            One PUB socket per camera, port defined in config.

        """
        super().__init__('zmq_bridge')
        self.get_logger().info('zmq_bridge_node')
        
        # Create the self.camera_config
        self.create_config()
        
        #CvBridge
        self.bridge = CvBridge()

        self.img_subscribers = {}
        self.latest_frames = {}

        # Create subscribers for each camera
        for name, cfg in self.camera_config.items():
            self.img_subscribers[name] = self.create_subscription(
                Image,
                cfg['topic'],
                lambda msg, n=name : self.img_callback(msg, n),
                10
            )

        # Single ZMQ socket shared across all cameras
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.zmq_port}")


    def create_config(self):
        """
        Declare and read ROS parameters to build the camera configuration.

        Args
        ----
        None

        Returns
        -------
        None

        """
        self.declare_parameter('cameras', rclpy.Parameter.Type.STRING_ARRAY)
        self.cameras = self.get_parameter('cameras').get_parameter_value().string_array_value

        self.camera_config = {}

        self.declare_parameter('zmq_port', rclpy.Parameter.Type.INTEGER)
        self.zmq_port = self.get_parameter('zmq_port').value

        for name in self.cameras:
            self.declare_parameter(f'{name}.topic', rclpy.Parameter.Type.STRING)
            self.declare_parameter(f'{name}.quality', rclpy.Parameter.Type.INTEGER)
            self.camera_config[name] = {
                'topic':   self.get_parameter(f'{name}.topic').get_parameter_value().string_value,
                'quality': self.get_parameter(f'{name}.quality').value,
            }

    def img_callback(self, msg, name):
        """
        Image callback to receive and forward camera frames via ZMQ.

        Args
        ----
        msg : sensor_msgs/msg/Image
            Incoming image message from the camera topic.

        name : str
            Camera name used to look up config and ZMQ socket.

        Returns
        -------
        None

        """

        cfg = self.camera_config[name]
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, cfg['quality']])
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.latest_frames[name] = {'ts': ts, 'jpg': jpg.tobytes()}

        if len(self.latest_frames) < len(self.camera_config):
            return

        payload = msgpack.packb(
            {
                'timestamps': {n: v['ts']  for n, v in self.latest_frames.items()},
                'images':     {n: v['jpg'] for n, v in self.latest_frames.items()},
            },
            use_bin_type=True,
        )
        self.socket.send(payload)


def main(args=None):
    """Entrypoint for zmq_bridge node."""
    rclpy.init(args=args)
    node = ZMQBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)