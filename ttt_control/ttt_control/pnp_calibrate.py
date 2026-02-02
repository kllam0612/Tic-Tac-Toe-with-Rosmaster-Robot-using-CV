import rclpy
from rclpy.node import Node
import sys
import time

# 3) Import custom message
from arm_msgs.msg import ArmJoints

class PnpCalibrate(Node):
    def __init__(self):
        # 1) Initialize Node
        super().__init__('pnp_calibrate')

        # 2) Publish to arm6_joints
        # Queue size 10
        self.publisher_ = self.create_publisher(ArmJoints, 'arm6_joints', 10)
        
        # 4) Internal memory: None
        
        # Define Poses (format: [j1, j2, j3, j4, j5, j6, time_ms])
        # Note: These floats will be rounded to match the int16 message definition
        self.POSES = {
            # 7) Initial Pose: Home
            'home':             [90, 90, 50, -40, 90, 141, 3000],
            
            # 8a) Supply Sequence Poses
            'supply_hover': [0, 60, 30, 0, 90, 141, 2500],
            'supply_pre_grasp': [0, 20, 50, 45, 90, 141, 2500],
            'supply_calibrate': [0, 0, 72, 21, 90, 141, 2500],
            
            # 8b) Board Sequence Poses
            'cell_1_pre_grasp': [110, 49, 18, 28, 72, 141, 2500],
            'cell_1_calibrate':  [108, 28, 17, 44, 72, 141, 2500],
            'cell_2_pre_grasp': [90, 53, 9, 32, 90, 141, 2500],
            'cell_2_calibrate':  [90, 36, 2, 52, 90, 141, 2500],
            'cell_3_pre_grasp': [70, 45, 17, 33, 108, 141, 2500],
            'cell_3_calibrate':  [69, 26, 18, 45, 108, 141, 2500],
            'cell_4_pre_grasp': [106, 15, 75, 5, 76, 141, 2500],
            'cell_4_calibrate':  [105, 1, 71, 19, 76, 141, 2500],
            'cell_5_pre_grasp': [90, 40, 35, 25, 90, 141, 2500],
            'cell_5_calibrate':  [90, 12, 47, 36, 90, 141, 2500],
            'cell_6_pre_grasp': [75, 19, 76, 0, 105, 141, 2500],
            'cell_6_calibrate':  [75, 0, 78, 14, 105, 141, 2500],
            'cell_7_pre_grasp': [104, 28, 56, 24, 79, 141, 3000],
            'cell_7_calibrate':  [102, 0, 72, 35, 79, 141, 2500],
            'cell_8_pre_grasp': [90, 28, 56, 24, 90, 140, 2500],
            'cell_8_calibrate':  [90, 9, 54, 41, 90, 141, 2500],
            'cell_9_pre_grasp': [77, 44, 19, 60, 99, 141, 2500],
            'cell_9_calibrate':  [79, 0, 73, 33, 105, 141, 2500],
        }
        
        self.get_logger().info("PnpCalibrate Node Initialized.")

    def publish_pose(self, pose_name):
        """
        Helper to construct the ArmJoints message and publish it.
        """
        if pose_name not in self.POSES:
            self.get_logger().error(f"Pose '{pose_name}' not found!")
            return

        data = self.POSES[pose_name]
        
        msg = ArmJoints()
        
        # Assigning joints: Rounding float data to int16 as per msg definition 
        msg.joint1 = int(round(data[0]))
        time.sleep(0.5)
        msg.joint2 = int(round(data[1]))
        time.sleep(0.5)
        msg.joint3 = int(round(data[2]))
        time.sleep(0.5)
        msg.joint4 = int(round(data[3]))
        time.sleep(0.5)
        msg.joint5 = int(round(data[4]))
        time.sleep(0.5)
        msg.joint6 = int(round(data[5]))
        
        # Assigning time (field name is 'time', type int16) 
        msg.time = int(data[6]) 

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {pose_name} -> {[msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]}")
        
        # Block for the duration of the movement to ensure sequentiality
        time.sleep(data[6] / 1000.0)

    def wait_for_confirmation(self):
        """
        Loops strictly until user inputs 'y'.
        """
        while True:
            # 8) Terminal prompt loop
            user_input = input("   [Terminal Prompt] Calibration ready? Key in 'y' to continue: ").strip().lower()
            if user_input == 'y':
                return
            else:
                print("   ... Waiting for 'y' ...")

    def run_supply_sequence(self):
        print("\n--- Starting Supply Calibration ---")
        self.publish_pose('supply_hover')
        self.publish_pose('supply_pre_grasp')
        self.publish_pose('supply_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('supply_pre_grasp')
        self.publish_pose('supply_hover')
        self.publish_pose('home')
        print("--- Supply Calibration Complete ---\n")

    def run_c1_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_1_pre_grasp')
        self.publish_pose('cell_1_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_1_pre_grasp')
        self.publish_pose('home')
        print("--- cell_1 Calibration Complete ---\n")

    def run_c2_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_2_pre_grasp')
        self.publish_pose('cell_2_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_2_pre_grasp')
        self.publish_pose('home')
        print("--- cell_2 Calibration Complete ---\n")
    
    def run_c3_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_3_pre_grasp')
        self.publish_pose('cell_3_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_3_pre_grasp')
        self.publish_pose('home')
        print("--- cell_3 Calibration Complete ---\n")
    
    def run_c4_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_4_pre_grasp')
        self.publish_pose('cell_4_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_4_pre_grasp')
        self.publish_pose('home')
        print("--- cell_4 Calibration Complete ---\n")
    
    def run_c5_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_5_pre_grasp')
        self.publish_pose('cell_5_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_5_pre_grasp')
        self.publish_pose('home')
        print("--- cell_5 Calibration Complete ---\n")
    
    def run_c6_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_6_pre_grasp')
        self.publish_pose('cell_6_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_6_pre_grasp')
        self.publish_pose('home')
        print("--- cell_6 Calibration Complete ---\n")
    
    def run_c7_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_7_pre_grasp')
        self.publish_pose('cell_7_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_7_pre_grasp')
        self.publish_pose('home')
        print("--- cell_7 Calibration Complete ---\n")

    def run_c8_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_8_pre_grasp')
        self.publish_pose('cell_8_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_8_pre_grasp')
        self.publish_pose('home')
        print("--- cell_8 Calibration Complete ---\n")
    
    def run_c9_sequence(self):
        print("\n--- Starting Board Calibration ---")
        self.publish_pose('cell_9_pre_grasp')
        self.publish_pose('cell_9_calibrate')
        
        # Loop until user confirm
        self.wait_for_confirmation()
        
        self.publish_pose('cell_9_pre_grasp')
        self.publish_pose('home')
        print("--- cell_9 Calibration Complete ---\n")

    def run_gui(self):
        """
        8) GUI for user to choose calibrate
        """
        # 7) Initial pose; home
        print("Moving to Initial Home Pose...")
        self.publish_pose('home')

        while True:
            print("===========================")
            print("   PNP CALIBRATION GUI     ")
            print("===========================")
            print("0) supply")
            print("9) cell 9")
            print("8) cell 8")
            print("7) cell 7")
            print("6) cell 6")
            print("5) cell 5")
            print("4) cell 4")
            print("3) cell 3")
            print("2) cell 2")
            print("1) cell 1")
            print("x) exit")
            
            choice = input("Select Option (x,0-9): ").strip()

            if choice == '0':
                self.run_supply_sequence()
            elif choice == '9':
                self.run_c9_sequence()
            elif choice == '8':
                self.run_c8_sequence()
            elif choice == '7':
                self.run_c7_sequence()
            elif choice == '6':
                self.run_c6_sequence()
            elif choice == '5':
                self.run_c5_sequence()
            elif choice == '4':
                self.run_c4_sequence()
            elif choice == '3':
                self.run_c3_sequence()
            elif choice == '2':
                self.run_c2_sequence()
            elif choice == '1':
                self.run_c1_sequence()
            elif choice == 'x':
                print("Exiting (kill process)...")
                break
            else:
                print("Invalid input. Please choose (x,0-9).")

def main(args=None):
    rclpy.init(args=args)
    
    # 1) Class PnpCalibrate, initialize as pnp_calibrate
    node = PnpCalibrate()

    # 5) Single thread execution
    try:
        node.run_gui()
    except KeyboardInterrupt:
        print("\nForce Quit detected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
