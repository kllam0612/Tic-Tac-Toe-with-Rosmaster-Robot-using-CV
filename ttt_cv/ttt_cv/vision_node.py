#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import sys
from ament_index_python.packages import get_package_share_directory
import os
import time
import json
import threading
from enum import Enum
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# these interfaces are generated in a package named 'ttt_interfaces'
from ttt_interfaces.msg import BoardState
from ttt_interfaces.srv import GetBoardState

class VisionState(Enum):
    VISION_IDLE = 0
    VISION_CALLED = 1
    VISION_PROCESS = 2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ---- Internal Memory ----
        self.latest_frame = None
        self.last_stamp = None
        # Board state is an array of 9 integers (0=Empty, 1=Robot, 2=Human)
        self.prev_board = [0] * 9
        self.current_board = [0] * 9
        
        # State Management
        self._state = VisionState.VISION_IDLE
        self._state_lock = threading.Lock() 

        # ---- Parameters ----
        self.declare_parameter('input_topic', 'image_raw')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('aruco_min_markers', 4)
        #self.declare_parameter('binary_threshold', 75)
        #self.declare_parameter('piece_split_area_ratio', 0.012)
        #self.declare_parameter('min_area1', 0.0005)
        #self.declare_parameter('max_area1', 0.035)

        self.input_topic = self.get_parameter('input_topic').value
        self.aruco_min_markers = int(self.get_parameter('aruco_min_markers').value)
        #self.binary_threshold = int(self.get_parameter('binary_threshold').value)
        #self.piece_split_ratio = float(self.get_parameter('piece_split_area_ratio').value)
        #self.min_area1 = float(self.get_parameter('min_area1').value)
        #self.max_area1 = float(self.get_parameter('max_area1').value)
        
        self.config_path = os.path.expanduser('~/ttt_ws/src/ttt_cv/config/camera_config.yaml')
    
        self._load_config()
        
        # ArUco Setup
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.aruco = cv2.aruco
        self.aruco_dict = self._get_aruco_dict(aruco_dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ---- ROS Infrastructure ----
        self.bridge = CvBridge()
        
        # Multi-threading Setup
        self.cb_group = ReentrantCallbackGroup()

        # Subscriber
        self.sub = self.create_subscription(
            Image, 
            self.input_topic, 
            self.image_callback, 
            10,
            callback_group=self.cb_group
        )

        # Service Server
        self.srv = self.create_service(
            GetBoardState,
            'get_board_state',
            self.handle_get_board_state,
            callback_group=self.cb_group
        )

        # Publisher (Reinstated)
        self.pub_state = self.create_publisher(String, '/ttt/board_state', 10)

        # ---- Storage for Debug Snaps ----
        self.pictures_dir = os.path.expanduser('~/Pictures/Debug')
        os.makedirs(self.pictures_dir, exist_ok=True)
        
        # Logging helper
        self.last_ascii_board = ""

        self.get_logger().info("[VISION_IDLE]")

    # ---------------------------------------------------------------------
    # Helper Methods
    # ---------------------------------------------------------------------
    def _load_config(self):
        """Load vision processing thresholds and stored board corners."""
        # Defaults
        self.binary_threshold = 80
        self.min_area1 = 150
        self.max_area1 = 300
        self.min_area2 = 750
        self.max_area2 = 1100

        try:
            with open(self.config_path, 'r') as file:
                data = yaml.safe_load(file) or {}

            ros_params = (data.get('camera_params') or {}).get('ros__parameters') or {}

            # Vision processing group
            proc_params = ros_params.get('vision_processing') or {}
            self.binary_threshold = int(proc_params.get('binary_threshold', self.binary_threshold))
            self.min_area1 = int(proc_params.get('min_area1', self.min_area1))
            self.max_area1 = int(proc_params.get('max_area1', self.max_area1))
            self.min_area2 = int(proc_params.get('min_area2', self.min_area2))
            self.max_area2 = int(proc_params.get('max_area2', self.max_area2))
            
            # Load corners from YAML
            calib = ros_params.get('geometry') or {}
            stored = calib.get('board_corners')
            if stored is not None and isinstance(stored, list) and len(stored) == 4:
                arr = np.array(stored, dtype=np.float32)
                if arr.shape == (4, 2):
                    self.median_board_corners = arr
                    self.get_logger().info(f"YAML Loaded board_corners:\n{self.median_board_corners}.")
            
            self.get_logger().info(f"YAML Loaded: Threshold: {self.binary_threshold}, Min_A1: {self.min_area1}, Max_A1: {self.max_area1}, Min_A2: {self.min_area2}, Max_A2: {self.max_area2}.")
        except Exception as e:
            self.get_logger().warn(f"Failed to load YAML ({self.config_path}): {e}. Using defaults.")

    def _get_aruco_dict(self, name: str):
        if not hasattr(cv2.aruco, name):
            self.get_logger().warn(f"Unknown ArUco dict {name}, using DICT_4X4_50")
            name = 'DICT_4X4_50'
        return self.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _now(self):
        return datetime.now().strftime('%Y%m%d_%H%M%S')

    def _save(self, img, prefix):
        path = os.path.join(self.pictures_dir, f"{prefix}_{self._now()}.png")
        cv2.imwrite(path, img)

    def _get_bbox_from_stats(self, stats_row):
        return (
            stats_row[cv2.CC_STAT_LEFT],
            stats_row[cv2.CC_STAT_TOP],
            stats_row[cv2.CC_STAT_WIDTH],
            stats_row[cv2.CC_STAT_HEIGHT]
        )

    # ---------------------------------------------------------------------
    # Core Logic: Service & Callback
    # ---------------------------------------------------------------------

    def image_callback(self, msg):
        """
        Lightweight callback.
        Branches on local snapshot of state.
        Only stores frame if state is VISION_CALLED.
        """
        local_state = self._state

        # VISION_IDLE: Ignore frames
        if local_state == VisionState.VISION_IDLE:
            return

        # VISION_CALLED: Read state and grab 1 frame
        if local_state == VisionState.VISION_CALLED:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.latest_frame = cv_img
                self.last_stamp = msg.header.stamp
            except Exception as e:
                self.get_logger().error(f"Image conversion failed: {e}")

    def handle_get_board_state(self, request, response):
        """
        Executes the logic pipeline.
        1. Reset logic (if requested)
        2. Transition to CALLED
        3. Wait for fresh frame
        4. Transition to PROCESS
        5. Run Pipeline
        6. Calculate Delta
        7. Transition to IDLE
        """
        #self.get_logger().info(f"GetBoardState called. Reset Tracker: {request.reset_diff_tracker}")

        # Handling Reset
        if request.reset_diff_tracker:
            self.prev_board = [0] * 9
            self.get_logger().info("Diff tracker reset: prev_board cleared.")

        # Transition State: IDLE -> CALLED
        with self._state_lock:
            self._state = VisionState.VISION_CALLED
            self.get_logger().info("[VISION_CALLED]")
        
        # Wait for a fresh frame (captured AFTER we set state to CALLED)
        timeout = 2.0 # seconds
        start_wait = time.time()
        captured = False
        
        current_op_stamp = self.last_stamp 

        while (time.time() - start_wait) < timeout:
            if self.last_stamp != current_op_stamp and self.latest_frame is not None:
                captured = True
                break
            time.sleep(0.05)

        if not captured:
            self.get_logger().error("Timed out waiting for camera frame.")
            with self._state_lock:
                self._state = VisionState.VISION_IDLE
            response.state.cells = self.prev_board
            response.state.last_move_cell = -1
            response.state.last_move_player = -1
            return response

        # Transition State: CALLED -> PROCESS
        with self._state_lock:
            self._state = VisionState.VISION_PROCESS
            self.get_logger().info("[VISION_PROCESS]")

        try:
            # Process the frozen self.latest_frame
            analyzed_board_dict, debug_img = self.execute_pipeline(self.latest_frame)
            
            # This maps Index 0 -> Cell 9 (Top Left), matching UI/Main Nodes.
            new_board_list = [0] * 9
            for i in range(9):
                cell_key = 9 - i  # 0->9, 1->8, ... 8->1
                val = analyzed_board_dict.get(cell_key, "EMPTY")
                
                if val == "O1": new_board_list[i] = 1
                elif val == "O2": new_board_list[i] = 2
                else: new_board_list[i] = 0

            self.current_board = new_board_list
            
            # Delta Verification
            delta_cell, delta_player = self.calculate_delta(self.current_board, self.prev_board)
            
            # Update history
            self.prev_board = list(self.current_board)

            # Update Response
            response.state.cells = self.current_board
            response.state.last_move_cell = delta_cell
            response.state.last_move_player = delta_player
            
            self.get_logger().info(f"Last Move: Cell {delta_cell}, Last Player {delta_player}")

        except Exception as e:
            self.get_logger().error(f"Pipeline processing error: {e}")
            import traceback
            traceback.print_exc()
            response.state.cells = self.prev_board
            response.state.last_move_cell = -1
            response.state.last_move_player = -1
        finally:
            # Transition State: PROCESS -> IDLE
            with self._state_lock:
                self._state = VisionState.VISION_IDLE
                self.get_logger().info("[VISION_IDLE]")

        return response

    def calculate_delta(self, current, previous):
        """
        Compare current vision result against previous confirmed state.
        Returns:
            (cell_index, player_id)
            cell_index: 1-9 for valid move, 0 if no change
            player_id: 1 or 2, 0 if no change
            Returns (-1, -1) on error/multiple moves
        """
        diffs = []
        for i in range(9):
            if current[i] != previous[i]:
                diffs.append((i, current[i]))
        
        if len(diffs) == 1:
            # Exactly one change detected
            cell_idx_0_based = diffs[0][0]
            new_val = diffs[0][1]
            
            # Valid move logic: Empty -> Player
            if previous[cell_idx_0_based] == 0 and new_val != 0:
                # -------------------------------------------------------------
                # Convert Index 0-based to Cell ID (1-9)
                # Since Array Index 0 = Cell 9, we must return (9 - index).
                # -------------------------------------------------------------
                return 9 - cell_idx_0_based, new_val
            else:
                self.get_logger().warn(f"Invalid state change: {previous[cell_idx_0_based]} -> {new_val}")
                return -1, -1
        elif len(diffs) == 0:
            # No change
            self.get_logger().warn(f"No changes detected")
            return 0, 0
        else:
            self.get_logger().warn(f"Multiple changes detected: {len(diffs)}")
            return -1, -1

    # ---------------------------------------------------------------------
    # Image Processing Pipeline
    # ---------------------------------------------------------------------

    def execute_pipeline(self, cv_bgr):
        """
        One-shot processing of a single frame.
        Returns: (board_state_dict, debug_image)
        """
        gray = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2GRAY)
        
        bw, bh = self._measure_wh(self.median_board_corners)
        dst_pts = np.array([[0, 0], [bw - 1, 0], [bw - 1, bh - 1], [0, bh - 1]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(self.median_board_corners, dst_pts)
        warped = cv2.warpPerspective(gray, M, (bw, bh))

        # 4. Pre-processing
        blurred = cv2.GaussianBlur(warped, (5, 5), 0)
        _, binary = cv2.threshold(blurred, self.binary_threshold, 255, cv2.THRESH_BINARY)
        self._save(binary, "debug_binary")
        
        # 5. Blob Cleaning
        binary_inv = cv2.bitwise_not(binary)
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        opened = cv2.morphologyEx(binary_inv, cv2.MORPH_OPEN, kernel_open)
        
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(opened)
        clean_mask = np.zeros_like(binary, dtype=np.uint8)
        
        # --- Improved Filtering Logic ---
        for i in range(1, num_labels):
             area = stats[i, cv2.CC_STAT_AREA]
             w_bb = stats[i, cv2.CC_STAT_WIDTH]
             h_bb = stats[i, cv2.CC_STAT_HEIGHT]
             
             is_o1 = (self.min_area1 < area < self.max_area1)
             is_o2 = (self.min_area2 < area < self.max_area2)

             # 1. Strict Area Check
             if is_o1 or is_o2:
        
              # 2. Tightened Aspect Ratio (Squareness)
              # Shadows are often long; pieces are roughly 1:1.
              # Reducing the tolerance from 0.4 to 0.2 filters out rectangles.
              aspect_ratio_diff = abs(w_bb - h_bb) / max(w_bb, h_bb)
        
              # 3. Circularity Check (The "Secret Sauce")
              # Formula: 4 * pi * Area / (Perimeter^2). 1.0 is a perfect circle.
              # This effectively kills the long, irregular shadow blobs.
              mask_i = (labels == i).astype(np.uint8) * 255
              contours, _ = cv2.findContours(mask_i, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
              
              if contours:
               perimeter = cv2.arcLength(contours[0], True)
               circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0
            
               # Combine filters: Square-ish AND Circular-ish
               if aspect_ratio_diff < 0.25 and circularity > 0.7:
                clean_mask[labels == i] = 255

        # Save debug snapshots
        final_clean = cv2.bitwise_not(clean_mask)
        self._save(final_clean, "debug_clean")

        # 6. Analyze Board (Returns state + debug image)
        board_state, debug_img = self._analyze_blobs(warped, clean_mask)
        self._save(debug_img, "debug_overlay")
        
        return board_state, debug_img

    def _analyze_blobs(self, warped_gray, binary_clean_mask):
        """
        1. Draws Grid
        2. Maps Blobs to Cells
        3. Classifies O1/O2
        4. Generates ASCII Art
        5. Publishes State
        Returns: (Dict, DebugImage)
        """
        h, w = warped_gray.shape
        debug_img = cv2.cvtColor(warped_gray, cv2.COLOR_GRAY2BGR)
        
        # 1. Draw Grid Lines (Cyan)
        cell_h = h // 3
        cell_w = w // 3
        cv2.line(debug_img, (0, cell_h), (w, cell_h), (255, 255, 0), 2)
        cv2.line(debug_img, (0, cell_h*2), (w, cell_h*2), (255, 255, 0), 2)
        cv2.line(debug_img, (cell_w, 0), (cell_w, h), (255, 255, 0), 2)
        cv2.line(debug_img, (cell_w*2, 0), (cell_w*2, h), (255, 255, 0), 2)
        
        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(binary_clean_mask)
        
        board_state = {i: "EMPTY" for i in range(1, 10)}

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            cx, cy = centroids[i]
            x, y, bw, bh = self._get_bbox_from_stats(stats[i])

            # 3. Map to Grid Cell
            col_idx = int(cx // cell_w)
            row_idx = int(cy // cell_h)
            col_idx = max(0, min(col_idx, 2))
            row_idx = max(0, min(row_idx, 2))
            cell_id = 9 - (row_idx * 3) - col_idx

            # 4. Classify 
            if self.min_area1 < area < self.max_area1:
                piece_type = "O1" 
                color = (0, 0, 255) # Red
                conf = 100
            elif self.min_area2 < area < self.max_area2:
                piece_type = "O2" 
                color = (0, 255, 0) # Green
                conf = 100
            else:
                piece_type = "N/A" 
                color = (255, 0, 0) # Blue
                conf = 100

            board_state[cell_id] = piece_type
            
            # Draw visuals
            cv2.rectangle(debug_img, (x, y), (x+bw, y+bh), color, 2)
            label_text = f"{piece_type} {conf}%"
            cv2.putText(debug_img, label_text, (x, y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Generate ASCII Board
        def get_sym(cid):
            p = board_state[cid]
            if p == "O1": return " O1 "
            if p == "O2": return " O2 "
            return f" {cid} "

        ascii_board = (
            f"\n+----+----+----+\n"
            f"|{get_sym(9)}|{get_sym(8)}|{get_sym(7)}|\n"
            f"+----+----+----+\n"
            f"|{get_sym(6)}|{get_sym(5)}|{get_sym(4)}|\n"
            f"+----+----+----+\n"
            f"|{get_sym(3)}|{get_sym(2)}|{get_sym(1)}|\n"
            f"+----+----+----+"
        )

        # Publish State (ASCII + JSON)
        msg_payload = {
            "timestamp": self._now(),
            "cells": board_state,
            "ascii": ascii_board
        }
        msg = String()
        msg.data = json.dumps(msg_payload)
        self.pub_state.publish(msg)

        # Log if changed
        if ascii_board != self.last_ascii_board:
            self.get_logger().info(f"Board State Updated:\n{ascii_board}")
            self.last_ascii_board = ascii_board

        return board_state, debug_img

    # ---------------------------------------------------------------------
    # Geometry Helpers
    # ---------------------------------------------------------------------

    def _pick_nearest_corner(self, marker, target):
        d = np.sum((marker - np.array(target, np.float32)) ** 2, axis=1)
        return marker[int(np.argmin(d))]

    def _compute_board_corners(self, corners, img_w, img_h):
        markers = [c.reshape(4, 2).astype(np.float32) for c in corners]
        centers = np.array([np.mean(m, axis=0) for m in markers])

        sums = centers[:, 0] + centers[:, 1]
        diffs = centers[:, 0] - centers[:, 1]

        tl_idx = int(np.argmin(sums))
        br_idx = int(np.argmax(sums))
        tr_idx = int(np.argmax(diffs))
        bl_idx = int(np.argmin(diffs))

        chosen_centers = np.array([
            centers[tl_idx], centers[tr_idx], 
            centers[br_idx], centers[bl_idx]
        ])
        board_center = np.mean(chosen_centers, axis=0)

        tl_pt = self._pick_nearest_corner(markers[tl_idx], board_center)
        tr_pt = self._pick_nearest_corner(markers[tr_idx], board_center)
        br_pt = self._pick_nearest_corner(markers[br_idx], board_center)
        bl_pt = self._pick_nearest_corner(markers[bl_idx], board_center)

        return np.array([tl_pt, tr_pt, br_pt, bl_pt], dtype=np.float32)

    def _measure_wh(self, pts):
        tl, tr, br, bl = pts
        w = int((np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2)
        h = int((np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2)
        return max(w, 20), max(h, 20)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

