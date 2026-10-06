"""ROS 2 node: publishes the QBot Platform RealSense D435 colour + depth images.

Topics:
  /camera/color/image_raw   sensor_msgs/Image  (bgr8)
  /camera/depth/image_raw   sensor_msgs/Image  (16UC1, millimetres)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from qbot_sensors.quanser_hw import QBotPlatformRealSense


def to_image_msg(array, encoding, stamp, frame_id):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = array.shape[0], array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = array.strides[0]
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('qbot_realsense')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        gp = lambda n: self.get_parameter(n).value
        self.frame_id = gp('frame_id')
        self.en_color = bool(gp('enable_color'))
        self.en_depth = bool(gp('enable_depth'))
        mode = ', '.join(m for m, on in (('RGB', self.en_color), ('Depth', self.en_depth)) if on)

        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)

        self.get_logger().info(f'Opening RealSense ({mode}) ...')
        self.cam = QBotPlatformRealSense(mode=mode, width=int(gp('width')),
                                         height=int(gp('height')), fps=float(gp('fps')))
        self.timer = self.create_timer(1.0 / (2.0 * float(gp('fps'))), self.loop)
        self.get_logger().info('Publishing /camera/color/image_raw and /camera/depth/image_raw')

    def loop(self):
        stamp = self.get_clock().now().to_msg()
        if self.en_color and self.cam.read_RGB() != -1:
            self.pub_color.publish(to_image_msg(self.cam.imageBufferRGB, 'bgr8', stamp, self.frame_id))
        if self.en_depth and self.cam.read_depth(dataMode='PX') != -1:
            depth = self.cam.imageBufferDepthPX[:, :, 0]
            self.pub_depth.publish(to_image_msg(depth, '16UC1', stamp, self.frame_id))

    def destroy_node(self):
        try:
            self.cam.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
