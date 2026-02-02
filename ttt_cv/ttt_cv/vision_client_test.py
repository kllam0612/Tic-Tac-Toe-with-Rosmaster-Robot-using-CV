#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from ttt_interfaces.srv import GetBoardState

class VisionClientTest(Node):

    def __init__(self):
        super().__init__('vision_client_test')
        self.cli = self.create_client(GetBoardState, 'get_board_state')
        
        self.get_logger().info('Waiting for "get_board_state" service...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service Connected!')

    def send_request(self, reset_tracker: bool):
        req = GetBoardState.Request()
        req.reset_diff_tracker = reset_tracker
        
        self.get_logger().info(f'Sending Request... (reset_diff_tracker={reset_tracker})')
        
        # Call service synchronously for this test script
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def print_board(state):
    """Helper to pretty-print the board state with Keypad Mapping"""
    cells = state.cells
    symbols = {0: '.', 1: 'R', 2: 'H'} # 1=Robot(X), 2=Human(O)
    
    print("\n--- Board State Response ---")
    print(f"Last Move Cell:   {state.last_move_cell}")
    
    player_str = 'None'
    if state.last_move_player == 1: player_str = 'Robot (X)'
    elif state.last_move_player == 2: player_str = 'Human (O)'
    print(f"Last Move Player: {state.last_move_player} ({player_str})")
    
    print("\nVisual Grid (Index 0 = Top-Left = Key 9):")
    
    # ---------------------------------------------------------------
    # LOGIC ALIGNMENT FIX:
    # Explicitly map array indices to the visual grid rows
    # Row 1: Indices 0, 1, 2 -> Keys 9, 8, 7
    # Row 2: Indices 3, 4, 5 -> Keys 6, 5, 4
    # Row 3: Indices 6, 7, 8 -> Keys 3, 2, 1
    # ---------------------------------------------------------------
    
    row1 = f" {symbols[cells[0]]} | {symbols[cells[1]]} | {symbols[cells[2]]} "
    row2 = f" {symbols[cells[3]]} | {symbols[cells[4]]} | {symbols[cells[5]]} "
    row3 = f" {symbols[cells[6]]} | {symbols[cells[7]]} | {symbols[cells[8]]} "
    div  = "---+---+---"
    
    print(f"{row1}   [Keys: 9 8 7]")
    print(f"{div}")
    print(f"{row2}   [Keys: 6 5 4]")
    print(f"{div}")
    print(f"{row3}   [Keys: 3 2 1]")
    print("----------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    client_node = VisionClientTest()

    print("\n=== Vision Test Client ===")
    print("Commands:")
    print("  'r' + Enter : Send Request with reset_diff_tracker = True (Start Game)")
    print("  'g' + Enter : Send Request with reset_diff_tracker = False (Get State)")
    print("  'q' + Enter : Quit")
    print("==========================\n")

    try:
        while rclpy.ok():
            try:
                # Basic input loop
                user_input = input("Enter command (r/g/q): ").strip().lower()
            except EOFError:
                break # Handle Ctrl+D

            if user_input == 'q':
                break
            
            reset_flag = False
            if user_input == 'r':
                reset_flag = True
            elif user_input == 'g' or user_input == '':
                reset_flag = False
            else:
                print("Unknown command. Use 'r', 'g', or 'q'.")
                continue

            try:
                response = client_node.send_request(reset_flag)
                if response:
                    print_board(response.state)
                else:
                    print("Error: No response received.")
            except Exception as e:
                print(f"Service call failed: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            client_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
