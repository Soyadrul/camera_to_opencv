import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import subprocess

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/output',  # o il topic corretto da camera_ros
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.last_saved_time = 0
        self.save_interval = 60  # secondi
        self.image_counter = 1
        self.output_dir = '/home/pi/camera_images'
        os.makedirs(self.output_dir, exist_ok=True)

    def listener_callback(self, msg):
        now = time.time()
        if now - self.last_saved_time >= self.save_interval:
            self.last_saved_time = now
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                image_path = os.path.join(self.output_dir, f'{self.image_counter}.jpg')
                cv2.imwrite(image_path, cv_image)
                self.get_logger().info(f'Saved image {image_path}')
                self.run_python_script(image_path)
                self.image_counter += 1
            except Exception as e:
                self.get_logger().error(f'Error saving image: {e}')

    def run_python_script(self, image_path):
        output_path = f'/home/pi/ros2_ws/src/wasr_deploy/output/predictions/out_{self.image_counter}.jpg'
        command = [
            'python3',
            '/home/pi/ros2_ws/src/wasr_deploy/predict_single.py',
            '--weight', '/home/pi/ros2_ws/src/wasr_deploy/weights/wasr_rn101.pth',
            image_path,
            output_path
        ]
        try:
            subprocess.run(command, check=True)
            self.get_logger().info(f'Processed image with WASR: {output_path}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error running WASR script: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
