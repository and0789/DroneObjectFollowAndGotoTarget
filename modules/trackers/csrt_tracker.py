"""
CSRT Tracker Implementation
===========================
Tracker menggunakan CSRT (Channel and Spatial Reliability Tracker).
OpenCV built-in tracker yang ringan dan akurat.

Kelebihan CSRT:
- Tidak perlu GPU
- Akurasi tinggi untuk single object
- Ringan untuk Raspberry Pi
- Built-in di OpenCV (tidak perlu install tambahan)

Kekurangan:
- Tidak bisa recover jika target hilang total
- Tidak ada re-detection seperti YOLO
"""

import cv2
import numpy as np
import time
import logging

from .base_tracker import BaseTracker

# Import config
import config


class CSRTTracker(BaseTracker):
    """
    CSRT (Channel and Spatial Reliability Tracker).
    
    Fitur:
    - Manual object selection (drag bbox)
    - Template-based recovery (seperti DaSiamRPN)
    - Adaptive tracking dengan channel reliability
    - Ringan untuk Raspberry Pi
    """
    
    def __init__(self):
        super().__init__()
        
        self.tracker = None
        self.model_available = True  # CSRT selalu available (OpenCV built-in)
        
        # Template untuk recovery
        self.object_template = None
        
        # Tracking parameters
        self.gamma = 0.02           # Learning rate untuk template update
        self.conf_thresh = 0.65     # Threshold untuk recovery (lebih rendah dari DaSiamRPN)
        self.search_scale = 2.5     # Scale factor untuk search area
        self.last_scan_score = 0.0
        
        # Tracking state
        self.consecutive_failures = 0
        self.max_failures_before_recovery = 3
        
        # Score simulation (CSRT tidak punya native score)
        self._tracking_quality = 1.0
        self._quality_decay = 0.02
    
    def initialize(self) -> bool:
        """
        Initialize CSRT tracker.
        CSRT adalah OpenCV built-in, jadi selalu berhasil.
        """
        try:
            # Test create tracker untuk memastikan OpenCV support CSRT
            test_tracker = cv2.TrackerCSRT_create()
            del test_tracker
            
            self.model_available = True
            logging.info("[CSRT] Tracker ready (OpenCV built-in)")
            return True
            
        except AttributeError:
            logging.error("[CSRT] TrackerCSRT not available in this OpenCV version")
            logging.error("[CSRT] Requires OpenCV >= 4.5 with contrib modules")
            self.model_available = False
            return False
    
    def _create_tracker(self):
        """Create new CSRT tracker instance."""
        return cv2.TrackerCSRT_create()
    
    def select_object(self, frame: np.ndarray) -> bool:
        """
        Interactive object selection dengan mouse drag.
        Sama seperti DaSiamRPN - user drag kotak di sekitar objek.
        """
        if not self.model_available:
            logging.error("[CSRT] Tracker not available")
            return False
        
        # Create new tracker instance
        self.tracker = self._create_tracker()
        self.is_tracking = False
        
        time.sleep(0.2)  # Brief pause untuk UI stability
        
        window_name = "SELECT TARGET (Drag & Press ENTER)"
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
            
            # Reset tracking quality
            self._tracking_quality = 1.0
            self.consecutive_failures = 0
            
            logging.info(f"[CSRT] Target selected: {w}x{h} pixels, "
                        f"ratio: {self.initial_bbox_ratio:.2%}")
            return True
        
        return False
    
    def select_object_programmatic(self, frame: np.ndarray, bbox: tuple) -> bool:
        """
        Programmatic object selection (tanpa GUI).
        Berguna untuk integrasi dengan YOLO detection atau auto-selection.
        
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
            
            self._tracking_quality = 1.0
            self.consecutive_failures = 0
            
            logging.info(f"[CSRT] Target set programmatically: {w}x{h} pixels")
            return True
        
        return False
    
    def update(self, frame: np.ndarray) -> tuple:
        """
        Update tracker dengan frame baru.
        
        Returns:
            tuple: (success, bbox, score)
                   score adalah estimasi kualitas tracking (0.0 - 1.0)
        """
        if not self.is_tracking or self.tracker is None:
            return False, None, 0.0
        
        # Update tracker
        success, bbox = self.tracker.update(frame)
        
        if success:
            # Validate bbox (kadang CSRT return bbox aneh)
            x, y, w, h = [int(v) for v in bbox]
            frame_h, frame_w = frame.shape[:2]
            
            # Check if bbox is valid
            if (w > 10 and h > 10 and 
                x >= 0 and y >= 0 and 
                x + w <= frame_w and y + h <= frame_h):
                
                self.last_valid_box = bbox
                self.consecutive_failures = 0
                
                # Update quality score
                self._tracking_quality = min(1.0, self._tracking_quality + 0.05)
                
                # Update template jika quality bagus
                if self._tracking_quality > 0.7:
                    self._update_template(frame, bbox)
                
                return True, bbox, self._tracking_quality
            else:
                # Invalid bbox
                success = False
        
        if not success:
            self.consecutive_failures += 1
            self._tracking_quality = max(0.0, self._tracking_quality - self._quality_decay)
            
            # Coba recovery jika sudah beberapa kali gagal
            if self.consecutive_failures >= self.max_failures_before_recovery:
                recovered, new_bbox, conf = self._try_recovery(frame)
                if recovered:
                    self.consecutive_failures = 0
                    self._tracking_quality = conf
                    return True, new_bbox, conf
            
            return False, None, self._tracking_quality
        
        return False, None, 0.0
    
    def _update_template(self, frame: np.ndarray, bbox: tuple):
        """Update template secara perlahan untuk adaptasi."""
        x, y, w, h = [int(v) for v in bbox]
        h_frm, w_frm = frame.shape[:2]
        
        # Clamp to frame bounds
        x = max(0, x)
        y = max(0, y)
        w = min(w, w_frm - x)
        h = min(h, h_frm - y)
        
        if w <= 0 or h <= 0:
            return
        
        current_view = frame[y:y+h, x:x+w].astype(np.float32)
        
        if self.object_template is not None:
            t_h, t_w = self.object_template.shape[:2]
            
            # Resize current view ke ukuran template
            try:
                current_view_resized = cv2.resize(current_view, (t_w, t_h))
                
                # Exponential moving average
                self.object_template = cv2.addWeighted(
                    self.object_template, (1 - self.gamma),
                    current_view_resized, self.gamma,
                    0
                )
            except:
                pass  # Skip jika resize gagal
    
    def _try_recovery(self, frame: np.ndarray) -> tuple:
        """
        Coba recover target yang hilang menggunakan template matching.
        Sama seperti DaSiamRPN recovery.
        """
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
            # Template matching
            res = cv2.matchTemplate(search_region, template_uint8, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(res)
            self.last_scan_score = max_val
            
            if max_val > self.conf_thresh:
                found_x = rx + max_loc[0]
                found_y = ry + max_loc[1]
                new_bbox = (found_x, found_y, self.obj_w, self.obj_h)
                
                # Re-initialize tracker dengan posisi baru
                self.tracker = self._create_tracker()
                self.tracker.init(frame, new_bbox)
                self.last_valid_box = new_bbox
                
                logging.info(f"[CSRT] Target recovered with confidence {max_val:.2f}")
                return True, new_bbox, max_val
                
        except Exception as e:
            logging.debug(f"[CSRT] Recovery error: {e}")
        
        return False, None, 0
    
    def release_target(self):
        """Release target dan reset state."""
        self.is_tracking = False
        self.tracker = None
        self.object_template = None
        self.last_valid_box = None
        self.consecutive_failures = 0
        self._tracking_quality = 1.0
        logging.info("[CSRT] Target released")
    
    def get_tracking_quality(self) -> float:
        """
        Get estimated tracking quality.
        
        Returns:
            float: Quality score 0.0 - 1.0
        """
        return self._tracking_quality
