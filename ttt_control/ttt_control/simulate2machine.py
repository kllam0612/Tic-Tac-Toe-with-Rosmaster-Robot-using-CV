import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
from arm_msgs.msg import ArmJoints
import tkinter as tk
import threading

class SimulateToMachine(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # Existing Arm Subscription
        self.sub_state = self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_group_controller/state",
            self.get_ArmPosCallback,
            1
        )

        # Gripper Subscription
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self.get_GripperPosCallback,
            10
        )

        self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
        
        # Initialize joints
        self.joints = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        
        # GUI Control Flags
        self.manual_mode = False
        self.slider_val = 90.0

    def start_gui(self):
        """Initializes and runs the Tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("Gripper Tuner")
        self.root.geometry("300x200")

        # Label
        lbl = tk.Label(self.root, text="Gripper Tuning (Offset Value)")
        lbl.pack(pady=10)

        # Checkbox for Manual Mode
        self.check_var = tk.BooleanVar()
        chk = tk.Checkbutton(self.root, text="Enable Manual Slider", variable=self.check_var, command=self.toggle_manual)
        chk.pack(pady=5)

        # Slider (90 to 180)
        self.slider = tk.Scale(self.root, from_=90, to=180, orient=tk.HORIZONTAL, length=250, command=self.on_slider_change)
        self.slider.set(90) # Default start
        self.slider.pack(pady=20)
        
        # Info Label
        self.val_label = tk.Label(self.root, text="Servo Output: --")
        self.val_label.pack(pady=5)

        self.root.mainloop()

    def toggle_manual(self):
        self.manual_mode = self.check_var.get()
        print(f"Manual Mode: {self.manual_mode}")

    def on_slider_change(self, val):
        if self.manual_mode:
            # Take slider value (acts as grip_val_offset)
            offset_input = float(val)
            
            # Apply the SAME mapping logic as the callback
            # Mapping [90, 180] -> [180, 90] (Reversed)
            servo_output = np.interp(offset_input, [90, 180], [180, 90])
            
            # Update Joints
            self.joints[5] = servo_output
            self.val_label.config(text=f"Servo Output: {int(servo_output)}")
            
            # Publish immediately
            self.pubSixArm(self.joints)

    def get_GripperPosCallback(self, msg):
        # IF Manual Mode is ON, ignore ROS updates for gripper
        if self.manual_mode:
            return

        target_joint_name = "rlink1_Joint"
        if target_joint_name in msg.name:
            try:
                index = msg.name.index(target_joint_name)
                grip_rad = msg.position[index]
                grip_deg = grip_rad * (180 / pi)
                
                # Calculation: -90 deg + 180 = 90; 0 deg + 180 = 180
                grip_val_offset = grip_deg + 180

                # REVERSED MAPPING: [90, 180] -> [180, 90]
                self.joints[5] = np.interp(grip_val_offset, [90, 180], [180, 90])
                
                self.pubSixArm(self.joints)
            except ValueError:
                pass

    def get_ArmPosCallback(self, msg):
        arm_rad = np.array(msg.actual.positions)
        DEG2RAD = np.array([180 / pi])
        arm_deg = np.dot(arm_rad.reshape(-1, 1), DEG2RAD)
        
        if len(msg.actual.positions) == 5:
            mid = np.array([90, 90, 90, 90, 90])
            arm_array = np.array(np.array(arm_deg) + mid)
            for i in range(5): 
                self.joints[i] = arm_array[i]
        
        elif len(msg.actual.positions) == 1:
            arm_array = np.array(np.array(arm_deg) + np.array([180]))
            # Legacy fallback
            self.joints[5] = np.interp(arm_array, [90, 180], [180, 90])[0]
        
        print("self.joints: ", self.joints)
        self.pubSixArm(self.joints)

    def pubSixArm(self, joints, id=6, angle=180.0, runtime=3000):
        arm_joints = ArmJoints()
        arm_joints.joint1 = 180- int(joints[0])
        arm_joints.joint2 = int(joints[1])
        arm_joints.joint3 = int(joints[2])
        arm_joints.joint4 = int(joints[3])
        arm_joints.joint5 = int(joints[4])
        arm_joints.joint6 = int(joints[5]) # Gripper
        arm_joints.time = runtime
        self.pub_SixTargetAngle.publish(arm_joints)

def main():
    rclpy.init()
    simulate2machine = SimulateToMachine('simulate2machine')
    
    # 1. Start ROS Node in a background thread so GUI doesn't freeze it
    ros_thread = threading.Thread(target=rclpy.spin, args=(simulate2machine,), daemon=True)
    ros_thread.start()
    
    # 2. Start GUI in the main thread
    try:
        simulate2machine.start_gui()
    except KeyboardInterrupt:
        pass
    finally:
        simulate2machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
