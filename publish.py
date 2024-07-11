import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'Occupancy_grid', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.bridge = CvBridge()

    def publish_image(self):
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'

        # Load your image from file (example)
        img_path = os.path.join(os.path.dirname(__file__), 'grid/grid.pgm')
        cv_image = cv2.imread(img_path, cv2.IMREAD_COLOR)

        # Convert cv2 image to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the image message
        self.publisher_.publish(img_msg)
        self.get_logger().info(f"Published image with timestamp {img_msg.header.stamp}")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
