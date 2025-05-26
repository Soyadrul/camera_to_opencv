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
        self.last_saved_time = 0
        self.save_interval = 50 # Seconds between 2 images
        self.image_counter = 1

    def listener_callback(self, msg):
        now = time.time()
        if now - self.last_saved_time >= self.save_interval:
            self.last_saved_time = now
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            image_to_process = self.cv_to_pil(cv_image)
            predict_ros_adaptation.predict(image_to_process)
            self.image_counter += 1
            
    def cv_to_pil(self, cv2_image):
        image_rgb = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
        return PIL.Image.fromarray(image_rgb)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
