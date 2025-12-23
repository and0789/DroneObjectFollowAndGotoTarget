"""
DaSiamRPN Tracker Implementation
================================
Tracker menggunakan DaSiamRPN (Distractor-aware Siamese Region Proposal Network).
Cocok untuk single object tracking dengan kecepatan tinggi.
"""

import cv2
import os
import numpy as np
import time
import logging

from .base_tracker import BaseTracker

import config


class DaSiamRPNTracker(BaseTracker):
    """
    DaSiamRPN Tracker dengan fitur:
    - Template adaptation
    - Recovery system menggunakan template matching
    - High-speed tracking
    """
    
    def __init__(self):
        super().__init__()
        
        self.tracker = None
        self.model_available = False
        self.params = None
        
        # Template untuk recovery
        self.object_template = None
        
        # Tracking parameters
        self.gamma = 0.01           # Learning rate untuk template update
        self.conf_thresh = 0.70     # Threshold untuk recovery
        self.search_scale = 3.0     # Scale factor untuk search area
        self.last_scan_score = 0.0
    
    def initialize(self) -> bool:
        """Load DaSiamRPN ONNX model."""
        model_exists = os.path.exists(config.MODEL_PATH)
        kernel_exists = os.path.exists(config.KERNEL_R1_PATH)
        
        if model_exists and kernel_exists:
            self.model_available = True
            self.params = cv2.TrackerDaSiamRPN_Params()
            self.params.model = config.MODEL_PATH
            self.params.kernel_cls1 = config.KERNEL_CLS1_PATH
            self.params.kernel_r1 = config.KERNEL_R1_PATH
            logging.info("[DaSiamRPN] Model loaded successfully")
            return True
        else:
            self.model_available = False
            logging.error(f"[DaSiamRPN] Model not found!")
            logging.error(f"  Expected: {config.MODEL_PATH}")
            logging.error(f"  Expected: {config.KERNEL_R1_PATH}")
            return False
    
    def _create_tracker(self):
        """Create new tracker instance."""
        return cv2.TrackerDaSiamRPN_create(self.params)
    
    def select_object(self, frame: np.ndarray) -> bool:
        """
        Interactive object selection dengan mouse drag.
        """
        if not self.model_available:
            logging.error("[DaSiamRPN] Model not available")
            return False
        
        self.tracker = self._create_tracker()
        self.is_tracking = False
        
        time.sleep(0.3)
        
        window_name = "SELECT TARGET (Drag & Enter)"
        bbox = cv2.selectROI(window_name, frame, False)
        cv2.destroyWindow(window_name)
        
        if bbox[2] > 0 and bbox[3] > 0:
            # Initialize tracker
            self.tracker.init(frame, bbox)
            self.is_tracking = True
            
            # Store bbox info
            x, y, w, h = [int(v) for v in bbox]
            self.last_valid_box = bbox
            self.obj_w = w
            self.obj_h = h
            
            # Calculate initial bbox ratio
            frame_h, frame_w = frame.shape[:2]
            self.initial_bbox_ratio = (w * h) / (frame_w * frame_h)
            
            # Extract template untuk recovery
            roi = frame[y:y+h, x:x+w]
            self.object_template = roi.astype(np.float32)
            
            logging.info(f"[DaSiamRPN] Target selected: {w}x{h} pixels, "
                        f"ratio: {self.initial_bbox_ratio:.2%}")
            return True
        
        return False
    
    def select_object_programmatic(self, frame: np.ndarray, bbox: tuple) -> bool:
        """
        Programmatic object selection (tanpa GUI).
        Berguna untuk integrasi dengan YOLO detection.
        
        Args:
            frame: Frame dari kamera
            bbox: (x, y, w, h) bounding box
        """
        if not self.model_available:
            return False
        
        self.tracker = self._create_tracker()
        
        x, y, w, h = [int(v) for v in bbox]
        
        if w > 0 and h > 0:
            self.tracker.init(frame, bbox)
            self.is_tracking = True
            
            self.last_valid_box = bbox
            self.obj_w = w
            self.obj_h = h
            
            frame_h, frame_w = frame.shape[:2]
            self.initial_bbox_ratio = (w * h) / (frame_w * frame_h)
            
            # Extract template
            y_end = min(y + h, frame_h)
            x_end = min(x + w, frame_w)
            roi = frame[y:y_end, x:x_end]
            if roi.size > 0:
                self.object_template = roi.astype(np.float32)
            
            return True
        
        return False
    
    def update(self, frame: np.ndarray) -> tuple:
        """
        Update tracker dengan frame baru.
        
        Returns:
            tuple: (success, bbox, score)
        """
        if not self.is_tracking or self.tracker is None:
            return False, None, 0.0
        
        success, bbox = self.tracker.update(frame)
        score = self.tracker.getTrackingScore()
        
        if success and score > 0.4:
            self.last_valid_box = bbox
            
            # Update template jika confidence tinggi
            if score > 0.6:
                self._update_template(frame, bbox)
            
            return True, bbox, score
        else:
            # Coba recovery
            recovered, new_bbox, conf = self._try_recovery(frame)
            if recovered:
                return True, new_bbox, conf
            
            return False, None, score
    
    def _update_template(self, frame: np.ndarray, bbox: tuple):
        """Update template secara perlahan untuk adaptasi."""
        x, y, w, h = [int(v) for v in bbox]
        h_frm, w_frm = frame.shape[:2]
        
        x = max(0, x)
        y = max(0, y)
        w = min(w, w_frm - x)
        h = min(h, h_frm - y)
        
        if w <= 0 or h <= 0:
            return
        
        current_view = frame[y:y+h, x:x+w].astype(np.float32)
        
        if self.object_template is not None:
            t_h, t_w = self.object_template.shape[:2]
            current_view_resized = cv2.resize(current_view, (t_w, t_h))
            
            self.object_template = cv2.addWeighted(
                self.object_template, (1 - self.gamma),
                current_view_resized, self.gamma,
                0
            )
    
    def _try_recovery(self, frame: np.ndarray) -> tuple:
        """Coba recover target yang hilang menggunakan template matching."""
        if self.last_valid_box is None or self.object_template is None:
            return False, None, 0
        
        lx, ly, lw, lh = [int(v) for v in self.last_valid_box]
        margin = int(max(lw, lh) * self.search_scale)
        
        h_frame, w_frame = frame.shape[:2]
        rx = max(0, lx - margin)
        ry = max(0, ly - margin)
        rw = min(lw + margin * 2, w_frame - rx)
        rh = min(lh + margin * 2, h_frame - ry)
        
        if rw <= self.obj_w or rh <= self.obj_h:
            return False, None, 0
        
        search_region = frame[ry:ry+rh, rx:rx+rw]
        template_uint8 = cv2.convertScaleAbs(self.object_template)
        
        try:
            res = cv2.matchTemplate(search_region, template_uint8, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)
            self.last_scan_score = max_val
            
            if max_val > self.conf_thresh:
                found_x = rx + max_loc[0]
                found_y = ry + max_loc[1]
                new_bbox = (found_x, found_y, self.obj_w, self.obj_h)
                
                # Re-initialize tracker
                self.tracker = self._create_tracker()
                self.tracker.init(frame, new_bbox)
                self.last_valid_box = new_bbox
                
                logging.info(f"[DaSiamRPN] Target recovered with confidence {max_val:.2f}")
                return True, new_bbox, max_val
        except Exception as e:
            logging.debug(f"[DaSiamRPN] Recovery error: {e}")
        
        return False, None, 0
    
    def release_target(self):
        """Release target dan reset state."""
        self.is_tracking = False
        self.tracker = None
        self.object_template = None
        self.last_valid_box = None
        logging.info("[DaSiamRPN] Target released")
