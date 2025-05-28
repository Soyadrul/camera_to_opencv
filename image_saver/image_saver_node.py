import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import subprocess
import PIL

username = os.environ.get('USERNAME') or os.environ.get('USER') or os.environ.get('LOGNAME') # Get the username of the current user
import sys
sys.path.append(f'/home/{username}/ros2_ws/src/wasr_deploy')
import predict_ros_adaptation

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # The correct camera_ros topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.processing = False  # Flag to indicate if processing is ongoing
        self.image_counter = 1
        self.output_dir = f'/home/{username}/camera_images' # Where to save the processed image
        os.makedirs(self.output_dir, exist_ok=True)

    def listener_callback(self, msg):
        
        if self.processing:
            # Skip this callback if still processing the previous image
            return
            
        self.processing = True
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Convert the image from a ROS2 message to a cv2 image
        image_to_process = self.cv_to_pil(cv_image) # Convert the cv2 image to a PIL image
        self.save_image(image_to_process) # Save the image in a folder (COMMENT THIS LINE IF YOU DON'T WANT TO SAVE THE IMAGE)
        self.process_image(image_to_process)
        self.processing = False
        
    def process_image(self, image):
        start = time.time()
        predict_ros_adaptation.predict(image, f"out-{self.image_counter}.jpg") # Call the function that will process the image
        end = time.time()
        elapsed = end - start
        self.get_logger().info(f"Inference time (image {self.image_counter}): {elapsed:.2f}s\n") # Show the time it took to process the image
        self.image_counter += 1
    
    # Function to convert the image from cv2 to a PIL image
    def cv_to_pil(self, cv2_image):
        image_rgb = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
        return PIL.Image.fromarray(image_rgb)
    
    # Function to save the image locally inside a folder
    def save_image(self, image):
        try:
            image_path = os.path.join(self.output_dir, f'{self.image_counter}.jpg')
            image.save(image_path)
            self.get_logger().info(f'Saved image {image_path}')
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
