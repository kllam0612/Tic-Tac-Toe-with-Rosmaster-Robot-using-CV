import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from arm_msgs.msg import ArmJoints
from cv_bridge import CvBridge
import cv2
import threading
import time
import subprocess
import signal
import os
import yaml
import sys

class CameraArmNode(Node):
    def __init__(self):
        super().__init__('camera_arm_node')

        # --- 1. Arm Configuration ---
        self.arm_pub = self.create_publisher(ArmJoints, 'arm6_joints', 10)
        self.init_joints = [90, 90, 50, -40, 90, 90]
        threading.Thread(target=self.move_arm_to_home).start()

        # --- 2. Camera Configuration ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw', 
            self.image_callback,
            10)
        
        # --- Publisher to send the tuned image to other nodes ---
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        
        # --- YAML LOADING (Modified) ---
        # Defines direct path to source file for easier editing without rebuilding
        self.config_path = os.path.expanduser('~/ttt_ws/src/ttt_cv/config/camera_config.yaml')
        
        try:
            with open(self.config_path, 'r') as file:
                data = yaml.safe_load(file) or {}

            # Safe extraction of nested keys
            ros_params = (data.get('camera_params') or {}).get('ros__parameters') or {}
            
            # Extract Camera Settings
            cam_settings = ros_params.get('camera_settings') or {}
            
            # Assign to internal variables with defaults (in case keys are missing)
            self.current_gain = cam_settings.get('gain', 0)
            self.current_contrast = cam_settings.get('contrast', 30)
            self.current_exposure = cam_settings.get('exposure', 600)
            self.current_brightness = cam_settings.get('brightness', 0)
            
            self.get_logger().info(f"Loaded Config: Exp={self.current_exposure}, Gain={self.current_gain}, Contrast={self.current_contrast}, Brightness={self.current_brightness}")

        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            # Fallback defaults if YAML fails
            self.current_gain = 0
            self.current_contrast = 30
            self.current_exposure = 600
            self.current_brightness = 0   
        
        # Process handle for the camera launch
        self.camera_process = None
        
        # Start the camera with initial defaults
        self.launch_camera()

    def launch_camera(self):
        """Launches the camera launch file as a subprocess with current args"""
        if self.camera_process:
            self.get_logger().info("Camera Arm Node Ready")
            # Kill the process group to ensure all child nodes die
            os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
            self.camera_process.wait()
            time.sleep(1.0) # Give it a moment to release USB

        # Build the command with current parameters
        cmd = [
            'ros2', 'launch', 'orbbec_camera', 'dabai_dcw2.launch.py',
            'camera_name:=camera',
            'enable_color_auto_exposure:=false',
            f'color_gain:={self.current_gain}',
            f'color_contrast:={self.current_contrast}',
            f'color_exposure:={self.current_exposure}',
            f'color_brightness:={self.current_brightness}'
        ]
        
        # Launch as a new process group so we can kill it cleanly later
        self.camera_process = subprocess.Popen(cmd, preexec_fn=os.setsid)

    def move_arm_to_home(self):
        time.sleep(2) 
        arm_msg = ArmJoints()
        arm_msg.joint1 = self.init_joints[0]
        arm_msg.joint2 = self.init_joints[1]
        arm_msg.joint3 = self.init_joints[2]
        arm_msg.joint4 = self.init_joints[3]
        arm_msg.joint5 = self.init_joints[4]
        arm_msg.joint6 = self.init_joints[5]
        arm_msg.time = 3000  
        self.arm_pub.publish(arm_msg)

    def image_callback(self, msg):
        try:
            # Publish the clean, tuned image for other nodes
            self.publisher_.publish(msg)
        except Exception as e:
            pass
            
def main(args=None):
    rclpy.init(args=args)
    node = CameraArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.camera_process:
             os.killpg(os.getpgid(node.camera_process.pid), signal.SIGTERM)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()