import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import argparse

class ImageConverter(Node):
    def __init__(self, input_topic, ouput_topic, encoding):
        super().__init__('image_converter_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.listener_callback,
            10)

        self.encoding = encoding
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Image, ouput_topic, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

        image_message = self.bridge.cv2_to_imgmsg(
            cv_image, 
            self.encoding
            # self.bridge.dtype_with_channels_to_cvtype2(cv_image.dtype, cv_image.ndim)
        )

        image_message.header.stamp = msg.header.stamp
        image_message.header.frame_id = msg.header.frame_id
        self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Convert compressed images to raw images.')
    parser.add_argument('input_topic', default="/oakd/rgb/image_raw/compressed", type=str, help='The name of the input topic.')
    parser.add_argument('output_topic', default="/oakd/rgb/image_raw", type=str, help='The name of the output topic.')
    parser.add_argument('encoding', default="bgr8", type=str, help='Type of encoding for output image')
    args = parser.parse_args()

    image_converter = ImageConverter(args.input_topic, args.output_topic, args.encoding)
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
