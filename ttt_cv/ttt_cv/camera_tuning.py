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
import yaml  # NEW: Required for exporting config

class camera_tuning(Node):
    def __init__(self):
        super().__init__('camera_tuning')

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
        
        # --- NEW: Publisher to send the tuned image to other nodes ---
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        
        # Internal variables
        self.current_gain = 0       
        self.current_contrast = 30    
        self.current_exposure = 600   
        self.current_brightness = 0   
        
        # --- NEW: Define Config Path ---
        # Adjust this path if your workspace structure differs
        self.yaml_path = os.path.expanduser('~/ttt_ws/src/ttt_cv/config/camera_config.yaml')
        
        try:
            with open(self.yaml_path, 'r') as file:
                data = yaml.safe_load(file) or {}

            # Safe extraction of nested keys
            ros_params = (data.get('camera_params') or {}).get('ros__parameters') or {}
            
            # Extract Camera Settings
            camera_settings = ros_params.get('camera_settings') or {}
            
            # Assign to internal variables with defaults (in case keys are missing)
            self.current_gain = camera_settings.get('gain', self.current_gain)
            self.current_contrast = camera_settings.get('contrast', self.current_contrast)
            self.current_exposure = camera_settings.get('exposure', self.current_exposure)
            self.current_brightness = camera_settings.get('brightness', self.current_brightness)
            
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

        self.get_logger().info('--- KEYBOARD CONTROL READY ---')
        self.get_logger().info('NOTE: Changing settings will RESTART the camera (2s delay)')
        self.get_logger().info('  [q] Exposure UP   |  [w] Exposure DOWN')
        self.get_logger().info('  [e] Gain UP       |  [r] Gain DOWN')
        self.get_logger().info('  [t] Contrast UP   |  [y] Contrast DOWN')
        self.get_logger().info('  [u] Brightness UP |  [i] Brightness DOWN')
        self.get_logger().info('  [s] SAVE current settings to YAML')
        self.get_logger().info('  [a] Quit')

    def launch_camera(self):
        """Launches the camera launch file as a subprocess with current args"""
        if self.camera_process:
            self.get_logger().info("Restarting Camera Node...")
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

    def save_params_to_yaml(self):
        """Reads existing YAML, updates camera settings, and saves back."""
        try:
            # Check if file exists to preserve other settings (like vision_processing)
            data = {}
            if os.path.exists(self.yaml_path):
                with open(self.yaml_path, 'r') as f:
                    try:
                        data = yaml.safe_load(f) or {}
                    except yaml.YAMLError:
                        data = {}
            
            # Ensure the hierarchy exists
            if 'camera_params' not in data: data['camera_params'] = {}
            if 'ros__parameters' not in data['camera_params']: data['camera_params']['ros__parameters'] = {}
            if 'camera_settings' not in data['camera_params']['ros__parameters']:
                data['camera_params']['ros__parameters']['camera_settings'] = {}

            # Update values
            settings = data['camera_params']['ros__parameters']['camera_settings']
            settings['gain'] = self.current_gain
            settings['contrast'] = self.current_contrast
            settings['exposure'] = self.current_exposure
            settings['brightness'] = self.current_brightness

            # Write back to file
            # Ensure dir exists
            os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
            with open(self.yaml_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            with open(self.yaml_path, 'r') as f:
                self.get_logger().info("Updated YAML:\n" + f.read())
            
        except Exception as e:
            self.get_logger().error(f"? Failed to save YAML: {e}")

    def image_callback(self, msg):
        try:
            # --- NEW: Publish the clean, tuned image for other nodes ---
            self.publisher_.publish(msg)

            # Convert ROS Image to OpenCV for display
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            status_text1 = f"Gain: {self.current_gain} | Contrast: {self.current_contrast}"
            status_text2 = f"Exp: {self.current_exposure} | Bright: {self.current_brightness}"
            status_text3 = "Press [s] to SAVE Config" # Visual reminder
            
            cv2.putText(cv_image, status_text1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(cv_image, status_text2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(cv_image, status_text3, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            cv2.imshow("Camera Tuning", cv_image)
            
            key = cv2.waitKey(1) & 0xFF
            updated = False
            
            # EXPOSURE (Limit: 1 to 2000)
            if key == ord('q'):
                self.current_exposure = min(self.current_exposure + 10, 1000) 
                updated = True
            elif key == ord('w'):
                self.current_exposure = max(self.current_exposure - 10, 1) 
                updated = True
                
            # GAIN (Limit: 0 to 100)
            elif key == ord('e'):
                self.current_gain = min(self.current_gain + 10, 100)
                updated = True
            elif key == ord('r'):
                self.current_gain = max(self.current_gain - 10, 0)
                updated = True
            
            # CONTRAST (Limit: 0 to 100)
            elif key == ord('t'):
                self.current_contrast = min(self.current_contrast + 10, 100)
                updated = True
            elif key == ord('y'):
                self.current_contrast = max(self.current_contrast - 10, 0)
                updated = True
            
            # BRIGHTNESS (Limit: -100 to 100)
            elif key == ord('u'):
                self.current_brightness = min(self.current_brightness + 10, 100)
                updated = True
            elif key == ord('i'):
                self.current_brightness = max(self.current_brightness - 10, -100)
                updated = True
            
            # SAVE TO YAML
            elif key == ord('s'):
                self.save_params_to_yaml()
            
            elif key == ord('a'):
                if self.camera_process:
                    os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                rclpy.shutdown()

            if updated:
                self.get_logger().info(f'Restarting with: G={self.current_gain}, C={self.current_contrast}, B={self.current_brightness}, E={self.current_exposure}')
                self.launch_camera() # Trigger restart

        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = camera_tuning()
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
