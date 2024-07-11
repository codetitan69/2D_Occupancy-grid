import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class MultiCameraDisplayNode(Node):
    def __init__(self):
        super().__init__('multi_camera_display_node')
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Declare and get parameters
        self.declare_parameter('camera1_topic', '/overhead_camera/overhead_camera1/image_raw')
        self.declare_parameter('camera2_topic', '/overhead_camera/overhead_camera2/image_raw')
        self.declare_parameter('camera3_topic', '/overhead_camera/overhead_camera3/image_raw')
        self.declare_parameter('camera4_topic', '/overhead_camera/overhead_camera4/image_raw')
        
        camera1_topic = self.get_parameter('camera1_topic').get_parameter_value().string_value
        camera2_topic = self.get_parameter('camera2_topic').get_parameter_value().string_value
        camera3_topic = self.get_parameter('camera3_topic').get_parameter_value().string_value
        camera4_topic = self.get_parameter('camera4_topic').get_parameter_value().string_value

        # Subscribe to the camera topics
        self.subscription1 = self.create_subscription(
            Image,
            camera1_topic,
            self.listener_callback1,
            10)
            
        self.subscription2 = self.create_subscription(
            Image,
            camera2_topic,
            self.listener_callback2,
            10)
            
        self.subscription3 = self.create_subscription(
            Image,
            camera3_topic,
            self.listener_callback3,
            10)
            
        self.subscription4 = self.create_subscription(
            Image,
            camera4_topic,
            self.listener_callback4,
            10)
            
        # Initialize images
        self.image1 = None
        self.image2 = None
        self.image3 = None
        self.image4 = None

        # Create a timer to periodically save the images
        self.timer = self.create_timer(0.1, self.save_images)

        # Create a directory to save images
        self.image_save_dir = 'saved_images'
        os.makedirs(self.image_save_dir, exist_ok=True)

    def listener_callback1(self, msg):
        self.image1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info('Received image from camera 1')

    def listener_callback2(self, msg):
        self.image2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info('Received image from camera 2')

    def listener_callback3(self, msg):
        self.image3 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info('Received image from camera 3')

    def listener_callback4(self, msg):
        self.image4 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info('Received image from camera 4')
        
    def save_images(self):
        if self.image1 is not None:
            cv2.imwrite(os.path.join(self.image_save_dir, 'image1.jpg'), self.image1)
        if self.image2 is not None:
            cv2.imwrite(os.path.join(self.image_save_dir, 'image2.jpg'), self.image2)
        if self.image3 is not None:
            cv2.imwrite(os.path.join(self.image_save_dir, 'image3.jpg'), self.image3)
        if self.image4 is not None:
            cv2.imwrite(os.path.join(self.image_save_dir, 'image4.jpg'), self.image4)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


