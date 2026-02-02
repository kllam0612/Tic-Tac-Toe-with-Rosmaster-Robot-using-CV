import sys
import threading
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from ttt_interfaces.action import ExecuteMove
from arm_msgs.msg import ArmJoints

class PnPTestNode(Node):
    def __init__(self):
        super().__init__('pnp_test_node')
        self._action_client = ActionClient(self, ExecuteMove, 'execute_move')
        self.get_logger().info("PnP Test UI Initialized. Waiting for Action Server...")
        
        # --- 1. Arm Configuration ---
        self.arm_pub = self.create_publisher(ArmJoints, 'arm6_joints', 10)
        self.init_joints = [90, 90, 50, -40, 90, 90]
        self.move_arm_to_home() 
  
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

    def send_goal(self, target_cell):
        """Sends the target cell to the action server."""
        
        # 1. Wait for Server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after 5 seconds.")
            return

        # 2. Create Goal Message
        goal_msg = ExecuteMove.Goal()
        goal_msg.target_cell = int(target_cell)

        self.get_logger().info(f"Sending goal: Target Cell {target_cell}...")

        # 3. Send Goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle whether the server accepted the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
            return

        self.get_logger().info("Goal accepted.")
        
        # 4. Wait for Result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the final result (Success/Fail)."""
        result = future.result().result
        status = future.result().status
        
        # Display Result to User
        if result.placed:
            self.get_logger().info(f"\n[RESULT] SUCCESS.")
        else:
            self.get_logger().error(f"\n[RESULT] FAILED.")
            
        # Signal the UI thread that we are done
        print("\n--- Ready for next command ---")

    def feedback_callback(self, feedback_msg):
        """Optional: Handle feedback if you implement it later."""
        # feedback = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: {feedback.log1}')
        pass

def user_input_thread(node):
    """Separate thread to handle blocking user input."""
    print("--- Pick and Place Control UI ---")
    print("Enter Target Cell Number (1-9) or 'q' to quit.")
    
    while rclpy.ok():
        try:
            user_in = input("Target Cell > ")
            if user_in.lower() == 'q':
                print("Exiting...")
                rclpy.shutdown()
                sys.exit(0)
            
            if user_in.isdigit() and 1 <= int(user_in) <= 9:
                # Dispatch the goal sending to the main ROS thread if needed, 
                # or call directly since send_goal is async-safe.
                node.send_goal(int(user_in))
            else:
                print("Invalid input. Please enter a number between 1 and 9.")
                
        except EOFError:
            break

def main(args=None):
    rclpy.init(args=args)
    
    node = PnPTestNode()
    
    # Run user input in a separate thread so it doesn't block ROS callbacks
    input_thread = threading.Thread(target=user_input_thread, args=(node,), daemon=True)
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Note: rclpy.shutdown() might be called by the thread already
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()