#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import sys
#from ament_index_python.packages import get_package_share_directory
import os
import time
import json
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class image_debug(Node):
    """
    Phase 0: Calibration (Median of 11 frames -> Freeze -> 'y' to save/next, 'n' to retry)
    Phase 1: Wait for Lighting ('y' to start)
    Phase 2: Processing (Warp -> Process -> Analyze -> Publish -> 's' to save, 'n' to recalib, 'y' to exit)
    Phase 3: Exit
    """

    def __init__(self):
        super().__init__('image_debug')

        # ---- State Machine ----
        self.phase = 0
        self.calib_buffer = [] # List of 11 items, each is np.array of shape (4,2) [TL, TR, BR, BL]
        self.median_board_corners = None
        
        # ---- Logging ----
        self._logged_callback_once = False
        self.last_ascii_board = "" 
        self.last_log_time = time.time()  # For throttling blob logs

        # ---- Display ----
        self.show_window = True
        self.window_name = "image_process - main"

        # ---- Parameters ----
        self.declare_parameter('input_topic', 'image_raw')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('aruco_min_markers', 4)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.aruco_min_markers = int(self.get_parameter('aruco_min_markers').value)

        # --- YAML LOADING ---
        self.config_path = os.path.expanduser('~/ttt_ws/src/ttt_cv/config/camera_config.yaml')
        
        self._load_config()

        # ---- ROS IO ----
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.pub_state = self.create_publisher(String, '/tic_tac_toe/board_state', 10)

        # ---- Storage ----
        self.pictures_dir = os.path.expanduser('~/Pictures/Debug')
        os.makedirs(self.pictures_dir, exist_ok=True)

        # ---- ArUco ----
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.aruco = cv2.aruco
        self.aruco_dict = self._get_aruco_dict(aruco_dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.get_logger().info("image_debug started. Entering Phase 0: Calibration.")

    # ---------------------------------------------------------------------
    # Utilities
    # ---------------------------------------------------------------------

    def _load_config(self):
        """Load vision processing thresholds (and optionally stored board corners)."""
        # Defaults
        self.binary_threshold = 80
        self.piece_split_ratio = 0.012
        self.min_area = 0.004
        self.max_area = 0.025

        try:
            with open(self.config_path, 'r') as file:
                data = yaml.safe_load(file) or {}

            ros_params = (data.get('camera_params') or {}).get('ros__parameters') or {}

            # Vision processing group
            proc_params = ros_params.get('vision_processing') or {}
            self.binary_threshold = int(proc_params.get('binary_threshold', self.binary_threshold))
            self.piece_split_ratio = float(proc_params.get('piece_split_area_ratio', self.piece_split_ratio))
            self.min_area = float(proc_params.get('min_area', self.min_area))
            self.max_area = float(proc_params.get('max_area', self.max_area))
            
            # Optional stored corners
            calib = ros_params.get('geometry') or {}
            stored = calib.get('board_corners')
            if stored is not None and isinstance(stored, list) and len(stored) == 4:
                arr = np.array(stored, dtype=np.float32)
                if arr.shape == (4, 2):
                    self.median_board_corners = arr
                    self.get_logger().info(f"YAML Loaded board_corners:\n{self.median_board_corners}.")
            
            self.get_logger().info(f"YAML Loaded: Threshold: {self.binary_threshold}, Ratio: {self.piece_split_ratio}, Min_A: {self.min_area}, Max_A: {self.max_area}.")
        except Exception as e:
            self.get_logger().warn(f"Failed to load YAML ({self.config_path}): {e}. Using defaults.")

    def _export_calibration_to_yaml(self, corners_4x2: np.ndarray) -> None:
        """Write median board corners into YAML under camera_params.ros__parameters.board_calibration."""
        try:
            data = {}
            if os.path.isfile(self.config_path):
                with open(self.config_path, 'r') as f:
                    data = yaml.safe_load(f) or {}

            data.setdefault('camera_params', {}).setdefault('ros__parameters', {})
            ros_params = data['camera_params']['ros__parameters']
            calib = ros_params.setdefault('geometry', {})

            calib['board_corners'] = [[float(x), float(y)] for x, y in corners_4x2.tolist()]
            

            with open(self.config_path, 'w') as f:
                yaml.safe_dump(data, f, sort_keys=False)

            with open(self.config_path, 'r') as f:
                self.get_logger().info("Updated YAML:\n" + f.read())

            self.get_logger().info(f"YAML Location: {os.path.abspath(self.config_path)}")
        except Exception as e:
            self.get_logger().error(f"Failed to export board corners to YAML: {e}")

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
        self.get_logger().info(f"Saved: {path}")

    def _get_bbox_from_stats(self, stats_row):
        return (
            stats_row[cv2.CC_STAT_LEFT],
            stats_row[cv2.CC_STAT_TOP],
            stats_row[cv2.CC_STAT_WIDTH],
            stats_row[cv2.CC_STAT_HEIGHT]
        )

    # ---------------------------------------------------------------------
    # Geometry & Math
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

        # Return ordered: TL, TR, BR, BL
        return np.array([tl_pt, tr_pt, br_pt, bl_pt], dtype=np.float32)

    def _measure_wh(self, pts):
        tl, tr, br, bl = pts
        w = int((np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2)
        h = int((np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2)
        return max(w, 20), max(h, 20)

    # ---------------------------------------------------------------------
    # Analysis Logic
    # ---------------------------------------------------------------------

    def analyze_board(self, warped_gray, binary_clean_mask):
        h, w = warped_gray.shape
        debug_img = cv2.cvtColor(warped_gray, cv2.COLOR_GRAY2BGR)

        # 1. Draw Grid Lines (Cyan)
        cell_h = h // 3
        cell_w = w // 3
        
        cv2.line(debug_img, (0, cell_h), (w, cell_h), (255, 255, 0), 2)
        cv2.line(debug_img, (0, cell_h*2), (w, cell_h*2), (255, 255, 0), 2)
        cv2.line(debug_img, (cell_w, 0), (cell_w, h), (255, 255, 0), 2)
        cv2.line(debug_img, (cell_w*2, 0), (cell_w*2, h), (255, 255, 0), 2)

        # 2. Component Analysis
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_clean_mask)
        
        board_state = {i: "EMPTY" for i in range(1, 10)}
        total_area = h * w
        
        min_area = self.min_area * total_area
        max_area = self.max_area * total_area
        split_threshold = self.piece_split_ratio * total_area

        # O1 Range: [min_area ... split_threshold]
        target_O1 = (min_area + split_threshold) / 2
        range_O1 = (split_threshold - min_area) / 2

        # O2 Range: [split_threshold ... max_area]
        target_O2 = (split_threshold + max_area) / 2
        range_O2 = (max_area - split_threshold) / 2

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            cx, cy = centroids[i]
            x, y, bw, bh = self._get_bbox_from_stats(stats[i])

            if area < min_area:
                continue

            # 3. Map to Grid Cell
            col_idx = int(cx // cell_w)
            row_idx = int(cy // cell_h)
            col_idx = max(0, min(col_idx, 2))
            row_idx = max(0, min(row_idx, 2))
            cell_id = 9 - (row_idx * 3) - col_idx

            # 4. Classify & Calculate Confidence
            if area < split_threshold:
                piece_type = "O1" 
                color = (0, 0, 255) # Red
                deviation = abs(area - target_O1)
                conf_float = max(0, 1.0 - (deviation / range_O1))
            else:
                piece_type = "O2" 
                color = (0, 255, 0) # Green
                deviation = abs(area - target_O2)
                conf_float = max(0, 1.0 - (deviation / range_O2))
            
            conf_percent = int(conf_float * 100)
            board_state[cell_id] = piece_type

            # 5. Draw Visuals
            cv2.rectangle(debug_img, (x, y), (x+bw, y+bh), color, 2)
            label_text = f"{piece_type} {conf_percent}%"
            cv2.putText(debug_img, label_text, (x, y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # 6. Generate ASCII Board
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

        # 7. Publish State
        msg_payload = {
            "timestamp": self._now(),
            "cells": board_state,
            "ascii": ascii_board
        }
        msg = String()
        msg.data = json.dumps(msg_payload)
        self.pub_state.publish(msg)

        if ascii_board != self.last_ascii_board:
            self.get_logger().info(f"Board State Changed:\n{ascii_board}")
            self.last_ascii_board = ascii_board

        return debug_img

    # ---------------------------------------------------------------------
    # Main Callback & Phase Machine
    # ---------------------------------------------------------------------

    def image_callback(self, msg):
        if not self._logged_callback_once:
            #self.get_logger().info("Receiving image stream")
            self._logged_callback_once = True

        cv_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        overlay = cv_bgr.copy()
        
        # Global Key Handler
        key = cv2.waitKey(1) & 0xFF

        # ----------------------------------------------------------
        # PHASE 0: CALIBRATION
        # ----------------------------------------------------------
        if self.phase == 0:
            cv2.putText(overlay, "PHASE 0: CALIBRATING (Looking for 4 markers)", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            if self.median_board_corners is None:
                # Still filling buffer
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )
                
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
                    
                    if len(corners) == 4 and len(self.calib_buffer) < 11:
                        # Compute corners for this frame
                        current_board_corners = self._compute_board_corners(corners, w, h)
                        self.calib_buffer.append(current_board_corners)
                        
                        percent = int((len(self.calib_buffer) / 11) * 100)
                        self.get_logger().info(f"Calibration Buffer: {len(self.calib_buffer)}/11 ({percent}%)")

                    # If buffer full, compute median
                    if len(self.calib_buffer) == 11:
                        self.get_logger().info("Buffer full. Computing Median...")
                        
                        # Stack: shape (11, 4, 2)
                        stack = np.array(self.calib_buffer)
                        
                        # Compute Median along axis 0 (across the 11 frames)
                        # Result shape: (4, 2)
                        self.median_board_corners = np.median(stack, axis=0).astype(np.float32)

                        self.get_logger().info(f"Median Corners Computed:\n{self.median_board_corners}")
            
            else:
                # Buffer full, median computed. Show Frozen State.
                # Draw the frozen polygon
                poly = self.median_board_corners.reshape(-1, 1, 2).astype(np.int32)
                cv2.polylines(overlay, [poly], True, (0, 0, 255), 3)

                # Show instructions
                cv2.putText(overlay, "LOCKED. 'y' to Accept, 'n' to Reset", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                # Show Warped Preview
                bw, bh = self._measure_wh(self.median_board_corners)
                
                dst_pts = np.array([[0, 0], [bw - 1, 0], [bw - 1, bh - 1], [0, bh - 1]], dtype=np.float32)
                M = cv2.getPerspectiveTransform(self.median_board_corners, dst_pts)
                warped_preview = cv2.warpPerspective(gray, M, (bw, bh))
                cv2.imshow("Warped Preview", warped_preview)

                # Input Handling
                if key == ord('y'):
                    self.get_logger().info("Calibration Accepted.")
                    self._export_calibration_to_yaml(self.median_board_corners)
                    print(f"Width: {bw}, Height: {bh}")
                    cv2.destroyWindow("Warped Preview")
                    self.phase = 1
                
                elif key == ord('n'):
                    self.get_logger().info("Calibration Rejected. Resetting.")
                    self.calib_buffer = []
                    self.median_board_corners = None
                    cv2.destroyWindow("Warped Preview")

        # ----------------------------------------------------------
        # PHASE 1: WAITING / LIGHTING
        # ----------------------------------------------------------
        elif self.phase == 1:
            # Re-draw the locked polygon so user sees we are calibrated
            if self.median_board_corners is not None:
                poly = self.median_board_corners.reshape(-1, 1, 2).astype(np.int32)
                cv2.polylines(overlay, [poly], True, (0, 0, 255), 2)

            cv2.putText(overlay, "PHASE 1: Adjust Lighting. 'y' to Process.", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            
            if key == ord('y'):
                self.get_logger().info("Starting Processing Phase.")
                self.phase = 2

        # ----------------------------------------------------------
        # PHASE 2: PROCESSING
        # ----------------------------------------------------------
        elif self.phase == 2:
            cv2.putText(overlay, "PHASE 2: RUNNING. 's':Save, 'n':Recalib, 'y':Exit", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # 1. Warp Current Frame using Stored Calibration
            bw, bh = self._measure_wh(self.median_board_corners)
            dst_pts = np.array([[0, 0], [bw - 1, 0], [bw - 1, bh - 1], [0, bh - 1]], dtype=np.float32)
            M = cv2.getPerspectiveTransform(self.median_board_corners, dst_pts)
            warped = cv2.warpPerspective(gray, M, (bw, bh))

            # 2. Pipeline
            blurred = cv2.GaussianBlur(warped, (3, 3), 0)
            _, binary = cv2.threshold(blurred, self.binary_threshold, 255, cv2.THRESH_BINARY)
            
            # 3. Blob Cleaning
            binary_inv = cv2.bitwise_not(binary) 
            kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            opened = cv2.morphologyEx(binary_inv, cv2.MORPH_OPEN, kernel_open)
            
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(opened)
            clean_mask = np.zeros_like(binary, dtype=np.uint8)

            img_total_area = bh * bw
            min_area_blob = self.min_area * img_total_area
            max_area_blob = self.max_area * img_total_area
            
            # --- Data Collection for Debugging ---
            pre_filter_areas = []
            post_filter_areas = []

            # --- Improved Filtering Logic ---
            for i in range(1, num_labels):
             area = stats[i, cv2.CC_STAT_AREA]
             w_bb = stats[i, cv2.CC_STAT_WIDTH]
             h_bb = stats[i, cv2.CC_STAT_HEIGHT]
             
             # Collect Raw Data
             pre_filter_areas.append(area)
             
             # 1. Strict Area Check
             if min_area_blob < area < max_area_blob:
        
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
                post_filter_areas.append(area)
            
            # --- Debug Logging (Throttled to 60s) ---
            if time.time() - self.last_log_time > 60.0:
                self.get_logger().info(
                    f"Blob Stats [Total: {len(pre_filter_areas)} -> Kept: {len(post_filter_areas)}]"
                )
                self.get_logger().info(f" -> Raw Areas: {pre_filter_areas}")
                self.get_logger().info(f" -> Kept Areas: {post_filter_areas}")
                self.last_log_time = time.time()

            final_clean = cv2.bitwise_not(clean_mask)

            # 4. Analyze
            analyzed_overlay = self.analyze_board(warped, clean_mask)

            # Show Analysis Window
            cv2.imshow("Analysis", analyzed_overlay)

            # Input Handling
            if key == ord('s'):
                self.get_logger().info("Saving Snapshots...")
                self._save(gray, "gray")
                self._save(warped, "warped")
                self._save(blurred, "blur")
                self._save(binary, "binary")
                self._save(binary_inv, "binary_inv")
                self._save(final_clean, "clean")
                self._save(analyzed_overlay, "analyzed")

            elif key == ord('n'):
                # Re-calibrate geometry
                self.get_logger().info("Returning to Calibration (Phase 0)...")
                cv2.destroyWindow("Analysis")
                self.phase = 0
                self.calib_buffer = []
                self.median_board_corners = None
            
            elif key == ord('y'):
                # Exit
                self.get_logger().info("Exit Requested.")
                self.phase = 3

        # ----------------------------------------------------------
        # PHASE 3: EXIT
        # ----------------------------------------------------------
        elif self.phase == 3:
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit()

        # Update Main Window
        if self.show_window and self.phase != 3:
            cv2.imshow(self.window_name, overlay)

def main():
    rclpy.init()
    node = image_debug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()
