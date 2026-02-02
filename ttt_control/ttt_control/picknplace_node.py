import sys
import time
import os
import yaml 
from ament_index_python.packages import get_package_share_directory # To find path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# 3) Import Custom Interface
from arm_msgs.msg import ArmJoints
from ttt_interfaces.action import ExecuteMove

# 1) Class PicknplaceNode
class PicknplaceNode(Node):
    def __init__(self):
        # 1) Initialize as picknplace_node
        super().__init__('picknplace_node')

        # --- YAML LOADING START; Dynamically find the path ---
        config_file = os.path.join(
            get_package_share_directory('ttt_control'),
            'config',
            'picknplace_config.yaml'
        )
        
        #self.get_logger().info(f'Loading Configuration from: {config_file}')

        # 2. Load and Parse
        try:
            with open(config_file, 'r') as file:
                data = yaml.safe_load(file)
                # Extract the inner dictionary matches your YAML structure
                self.manual_params = data['picknplace_node']['ros__parameters']
                self.get_logger().info("YAML Loaded Successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            sys.exit(1)
        # --- [NEW] MANUAL YAML LOADING END ---

        # 2) Publish to arm6_joints
        self.publisher_ = self.create_publisher(ArmJoints, 'arm6_joints', 10)

        # 4) Create Action Server ExecuteMove
        self._action_server = ActionServer(
            self,
            ExecuteMove,
            'execute_move',
            self.handle_execute_move
        )

        self.get_logger().info('Picknplace Node Ready.')

        # --- INITIALIZATION START ---
        # 1. Define Internal State (Matches Physical Start Position)
        self.current_joints = [90, 90, 50, -40, 90, 90]

        # Wait for connections to be established
        #time.sleep(1.0) 

    def get_pose_param(self, param_path_str):
        """
        Traverses the self.manual_params dictionary using a dot-notation string.
        Example: 'targets.cell_1.down' -> self.manual_params['targets']['cell_1']['down']
        """
        keys = param_path_str.split('.')
        value = self.manual_params
        try:
            for k in keys:
                value = value[k]
            return value
        except KeyError:
            self.get_logger().error(f"Parameter not found in YAML: {param_path_str}")
            return [0]*7

    def publish_pose(self, pose_data):
        """
        Helper to output pose values for debugging.
        Does NOT publish to the robot.
        """
        # Convert to standard list for clean printing
        values = list(pose_data)
        
        # --- DEBUG OUTPUT ---
        #self.get_logger().info(f'   [YAML DATA LOADED]: {values}')
        
        msg = ArmJoints()
        msg.joint1 = int(pose_data[0])
        #time.sleep(0.5)
        msg.joint2 = int(pose_data[1])
        #time.sleep(0.5)
        msg.joint3 = int(pose_data[2])
        #time.sleep(0.5)
        msg.joint4 = int(pose_data[3])
        #time.sleep(0.5)
        msg.joint5 = int(pose_data[4])
        #time.sleep(0.5)
        msg.joint6 = int(pose_data[5])
        msg.time   = int(pose_data[6])
        
        self.current_joints = [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6]

        self.publisher_.publish(msg)
        time.sleep(4)

    # 10) handle_execute_move
    def handle_execute_move(self, goal_handle):
        
        # 10a) Receive target_cell
        target_cell = goal_handle.request.target_cell
        self.get_logger().info(f"Received Goal: {target_cell}.")
        
        # Validation
        if target_cell < 1 or target_cell > 9:
            self.get_logger().error(f'Invalid target cell: {target_cell}')
            goal_handle.abort()
            return ExecuteMove.Result(placed=False)

        # [MODIFIED] Access from dictionary directly
        gripper_close = self.manual_params['gripper_close']
        
        # --- Supply Sequence ---
        #self.get_logger().info('--- STEP: Supply Hover ---')
        self.publish_pose(self.get_pose_param('supply_sequence.supply_hover_1'))
        
        #self.get_logger().info('--- STEP: Supply Pre-Grasp 1 ---')
        self.publish_pose(self.get_pose_param('supply_sequence.pre_grasp_1'))
        time.sleep(1)

        #self.get_logger().info('--- STEP: Supply Down ---')
        self.publish_pose(self.get_pose_param('supply_sequence.down'))
        time.sleep(1)
        """
        # 10d) Terminal Prompt
        print("\n--- AWAITING USER INPUT ---")
        while True:
            user_input = input("Press 'y' to continue debug sequence: ")
            if user_input.lower() == 'y':
                break
        """
        # 10e) Supply Close
        #self.get_logger().info('--- STEP: Supply Close ---')
        pose_down = list(self.get_pose_param('supply_sequence.down'))
        pose_down[5] = gripper_close 
        pose_down[6] = 2500 
        self.publish_pose(pose_down)

        #self.get_logger().info('--- STEP: Supply Pre-Grasp 2 ---')
        self.publish_pose(self.get_pose_param('supply_sequence.pre_grasp_2'))
  
        #self.get_logger().info('--- STEP: Supply Hover ---')
        self.publish_pose(self.get_pose_param('supply_sequence.supply_hover_2'))
  
        #self.get_logger().info('--- STEP: Board Hover ---')
        self.publish_pose(self.get_pose_param('supply_sequence.board_hover'))

        # --- Target Sequence ---
        #self.get_logger().info(f"Processing Target Cell {target_cell}...")
        base_param = f'targets.cell_{target_cell}'
        
        #self.get_logger().info(f'--- STEP: Cell {target_cell} Pre-Grasp 1 ---')
        self.publish_pose(self.get_pose_param(f'{base_param}.pre_grasp_1'))
        time.sleep(2)
        
        #self.get_logger().info(f'--- STEP: Cell {target_cell} Down ---')
        self.publish_pose(self.get_pose_param(f'{base_param}.down'))
        time.sleep(2)
        
        #self.get_logger().info(f'--- STEP: Cell {target_cell} Open ---')
        self.publish_pose(self.get_pose_param(f'{base_param}.open'))
        
        #self.get_logger().info(f'--- STEP: Cell {target_cell} Pre-Grasp 2 ---')
        self.publish_pose(self.get_pose_param(f'{base_param}.pre_grasp_2'))

        # 10j) Return to home
        #self.get_logger().info('--- STEP: Return Home ---')
        self.publish_pose(self.get_pose_param('pose_home'))

        # 10k) Return Result
        goal_handle.succeed()
        result = ExecuteMove.Result()
        result.placed = True
        self.get_logger().info(f"Goal Success: {target_cell}.")
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PicknplaceNode()

    # 6) Single thread execution
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
