# Image Subscriber

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/unitycamera/occupancygridmap',  # Replace with your image topic
            self.image_callback,
            10)
        self.subscription  
        self.bridge = CvBridge()
        self.image_counter = 0

    def image_callback(self, msg):
        # ImageMsg to OpenCV Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # Show the image
        # print(cv_image.shape)
        self.show_image(cv_image) 



    def save_image(self,image):
        # Create a directory to save images if it doesn't exist
        save_dir = 'saved_images'
        os.makedirs(save_dir, exist_ok=True)

        # Save the image with a timestamp as the filename
        filename = os.path.join(save_dir, f"image_{rclpy.clock.Clock().now().nanoseconds}.png")
        cv2.imwrite(filename, image)
        self.get_logger().info(f"Image saved: {filename}")
    
    def show_image(self,image):
        # Image show
        cv2.imshow('Map', image)

        # Wait for a key event for a short duration (e.g., 1 millisecond)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') or cv2.getWindowProperty('Map', cv2.WND_PROP_VISIBLE) < 1:
            self.save_image(image) # Save Image

        if key == ord('q') or cv2.getWindowProperty('Map', cv2.WND_PROP_VISIBLE) < 1:
            cv2.destroyAllWindows()  # Destroy all OpenCV windows

        # Optionally, rclpy.shutdown() here to stop the ROS node
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber_node = ImageSubscriberNode()
    rclpy.spin(image_subscriber_node)
    image_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

