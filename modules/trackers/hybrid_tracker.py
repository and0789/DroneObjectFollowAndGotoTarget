"""
Hybrid Tracker Implementation
=============================
Kombinasi YOLO Detection + CSRT Tracking untuk performa optimal.

Strategi:
- YOLO: Detect setiap N frame (default: 10 frame)
- CSRT: Track di frame lainnya (ringan)
- Re-detection otomatis jika CSRT kehilangan target

Keuntungan:
- FPS tinggi (CSRT ringan)
- Akurasi tinggi (YOLO re-verify)
- Recovery otomatis (YOLO re-detect)
- Siap untuk Hailo acceleration di masa depan

Usage:
    tracker = HybridTracker(yolo_model="yolo11n.pt", detect_interval=10)
    tracker.initialize()
    tracker.select_object(frame)  # atau select_by_class(frame, "person")
    success, bbox, score = tracker.update(frame)
"""

import cv2
import numpy as np
import time
import logging

from .base_tracker import BaseTracker

# Import config
import config


class HybridTracker(BaseTracker):
    """
    Hybrid Tracker: YOLO Detection + CSRT Tracking.
    
    Fitur:
    - YOLO detection setiap N frame
    - CSRT tracking di frame lainnya
    - Auto re-detection jika tracking lost
    - Support untuk select by class (person, car, dll)
    - Prepared untuk Hailo acceleration
    """
    
    # COCO classes
    COCO_CLASSES = {
        0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
        5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
        10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
        14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
        20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
        25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
    }
    
    def __init__(self, yolo_model: str = "yolo11n.pt", 
                 detect_interval: int = 10,
                 target_classes: list = None,
                 use_hailo: bool = False):
        """
        Initialize Hybrid Tracker.
        
        Args:
            yolo_model: Path ke YOLO model (.pt file)
            detect_interval: Jalankan YOLO setiap N frame
            target_classes: Filter class untuk detection (None = semua)
            use_hailo: Gunakan Hailo accelerator (future feature)
        """
        super().__init__()
        
        # YOLO settings
        self.yolo_model_path = yolo_model
        self.yolo_model = None
        self.detect_interval = detect_interval
        self.target_classes = target_classes
        self.use_hailo = use_hailo
        
        # CSRT tracker
        self.csrt_tracker = None
        
        # State
        self.frame_count = 0
        self.tracked_class = None
        self.tracked_id = None
        self.all_detections = []
        
        # Confidence thresholds
        self.yolo_conf_threshold = 0.5
        self.csrt_quality_threshold = 0.3
        self.iou_threshold = 0.3  # Untuk matching YOLO dengan CSRT
        
        # Tracking quality
        self._csrt_quality = 1.0
        self._quality_decay = 0.02
        
        # Recovery settings
        self.consecutive_csrt_failures = 0
        self.max_csrt_failures = 5  # Setelah ini, paksa YOLO detect
        
        # Model availability
        self.model_available = False
    
    def initialize(self) -> bool:
        """Initialize YOLO model dan CSRT."""
        try:
            # Initialize YOLO
            if self.use_hailo:
                # TODO: Hailo YOLO initialization
                logging.info("[HYBRID] Hailo mode - not implemented yet, using CPU")
            
            from ultralytics import YOLO
            logging.info(f"[HYBRID] Loading YOLO model: {self.yolo_model_path}")
            self.yolo_model = YOLO(self.yolo_model_path)
            
            # Test CSRT availability
            test_csrt = cv2.TrackerCSRT_create()
            del test_csrt
            
            self.model_available = True
            logging.info(f"[HYBRID] Initialized successfully")
            logging.info(f"[HYBRID] YOLO detect every {self.detect_interval} frames")
            logging.info(f"[HYBRID] Target classes: {self.target_classes or 'ALL'}")
            return True
            
        except ImportError as e:
            logging.error(f"[HYBRID] Missing dependency: {e}")
            logging.error("[HYBRID] Install with: pip install ultralytics --break-system-packages")
            return False
            
        except Exception as e:
            logging.error(f"[HYBRID] Initialization failed: {e}")
            return False
    
    def _create_csrt(self):
        """Create new CSRT tracker instance (Safe for OpenCV 4+)."""
        # Coba akses via module legacy (untuk OpenCV 4.5 ke atas)
        if hasattr(cv2, 'legacy'):
            return cv2.legacy.TrackerCSRT_create()
        # Coba akses langsung (untuk OpenCV versi lama)
        elif hasattr(cv2, 'TrackerCSRT_create'):
            return cv2.TrackerCSRT_create()
        else:
            raise AttributeError("CSRT Tracker tidak ditemukan! Pastikan opencv-contrib-python terinstall.")
    
    def _run_yolo_detection(self, frame: np.ndarray) -> list:
        """
        Run YOLO detection pada frame.
        
        Returns:
            list: List of detections
        """
        if self.yolo_model is None:
            return []
        
        try:
            results = self.yolo_model(
                frame,
                conf=self.yolo_conf_threshold,
                verbose=False
            )
            
            detections = []
            
            for result in results:
                if result.boxes is None:
                    continue
                
                boxes = result.boxes
                
                for i in range(len(boxes)):
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    x1, y1, x2, y2 = xyxy
                    w = x2 - x1
                    h = y2 - y1
                    bbox = (int(x1), int(y1), int(w), int(h))
                    
                    class_id = int(boxes.cls[i].cpu().numpy())
                    confidence = float(boxes.conf[i].cpu().numpy())
                    class_name = self.COCO_CLASSES.get(class_id, f"class_{class_id}")
                    
                    # Filter by target classes
                    if self.target_classes is not None:
                        if class_id not in self.target_classes and class_name not in self.target_classes:
                            continue
                    
                    detections.append({
                        'bbox': bbox,
                        'class_id': class_id,
                        'class_name': class_name,
                        'confidence': confidence
                    })
            
            self.all_detections = detections
            return detections
            
        except Exception as e:
            logging.error(f"[HYBRID] YOLO detection error: {e}")
            return []
    
    def _calculate_iou(self, box1: tuple, box2: tuple) -> float:
        """Calculate IoU between two bboxes (x, y, w, h format)."""
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2
        
        # Convert to x1, y1, x2, y2
        box1_x2, box1_y2 = x1 + w1, y1 + h1
        box2_x2, box2_y2 = x2 + w2, y2 + h2
        
        # Intersection
        inter_x1 = max(x1, x2)
        inter_y1 = max(y1, y2)
        inter_x2 = min(box1_x2, box2_x2)
        inter_y2 = min(box1_y2, box2_y2)
        
        if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
            return 0.0
        
        inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
        
        # Union
        box1_area = w1 * h1
        box2_area = w2 * h2
        union_area = box1_area + box2_area - inter_area
        
        if union_area <= 0:
            return 0.0
        
        return inter_area / union_area
    
    def _find_matching_detection(self, csrt_bbox: tuple, detections: list) -> dict:
        """
        Cari detection yang match dengan CSRT bbox.
        
        Returns:
            dict: Matching detection atau None
        """
        if not detections or csrt_bbox is None:
            return None
        
        best_match = None
        best_iou = self.iou_threshold
        
        for det in detections:
            # Filter by class jika tracked_class di-set
            if self.tracked_class is not None:
                if det['class_id'] != self.tracked_class:
                    continue
            
            iou = self._calculate_iou(csrt_bbox, det['bbox'])
            if iou > best_iou:
                best_iou = iou
                best_match = det
        
        return best_match
    
    def select_object(self, frame: np.ndarray) -> bool:
        """
        Interactive object selection.
        Jalankan YOLO dulu, tampilkan detections, user pilih.
        """
        if not self.model_available:
            return False
        
        # Run YOLO detection
        detections = self._run_yolo_detection(frame)
        
        if not detections:
            logging.warning("[HYBRID] No objects detected by YOLO")
            # Fallback ke manual selection dengan CSRT
            return self._manual_select(frame)
        
        # Display detections dengan nomor
        display = frame.copy()
        for i, det in enumerate(detections):
            x, y, w, h = det['bbox']
            color = (0, 255, 0)
            cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
            label = f"{i}: {det['class_name']} ({det['confidence']:.2f})"
            
            # Background untuk text
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(display, (x, y - th - 10), (x + tw, y), color, -1)
            cv2.putText(display, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Instructions
        cv2.putText(display, "Press 0-9 to select, 'M' for manual, ESC to cancel",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("SELECT TARGET", display)
        
        while True:
            key = cv2.waitKey(0) & 0xFF
            
            if key == 27:  # ESC
                cv2.destroyWindow("SELECT TARGET")
                return False
            
            if key == ord('m') or key == ord('M'):
                # Manual selection
                cv2.destroyWindow("SELECT TARGET")
                return self._manual_select(frame)
            
            if ord('0') <= key <= ord('9'):
                idx = key - ord('0')
                if idx < len(detections):
                    cv2.destroyWindow("SELECT TARGET")
                    return self._start_tracking(frame, detections[idx])
        
        return False
    
    def _manual_select(self, frame: np.ndarray) -> bool:
        """Manual bbox selection seperti CSRT/DaSiamRPN."""
        self.csrt_tracker = self._create_csrt()
        
        window_name = "MANUAL SELECT (Drag & Press ENTER)"
        bbox = cv2.selectROI(window_name, frame, False)
        cv2.destroyWindow(window_name)
        
        if bbox[2] > 0 and bbox[3] > 0:
            self.csrt_tracker.init(frame, bbox)
            self.is_tracking = True
            
            x, y, w, h = [int(v) for v in bbox]
            self.last_valid_box = bbox
            self.obj_w = w
            self.obj_h = h
            
            frame_h, frame_w = frame.shape[:2]
            self.initial_bbox_ratio = (w * h) / (frame_w * frame_h)
            
            self.tracked_class = None  # Unknown class
            self._csrt_quality = 1.0
            self.frame_count = 0
            self.consecutive_csrt_failures = 0
            
            logging.info(f"[HYBRID] Manual selection: {w}x{h} pixels")
            return True
        
        return False
    
    def _start_tracking(self, frame: np.ndarray, detection: dict) -> bool:
        """Start tracking dari YOLO detection."""
        self.csrt_tracker = self._create_csrt()
        
        bbox = detection['bbox']
        x, y, w, h = bbox
        
        self.csrt_tracker.init(frame, bbox)
        self.is_tracking = True
        
        self.last_valid_box = bbox
        self.obj_w = w
        self.obj_h = h
        
        frame_h, frame_w = frame.shape[:2]
        self.initial_bbox_ratio = (w * h) / (frame_w * frame_h)
        
        self.tracked_class = detection['class_id']
        self._csrt_quality = 1.0
        self.frame_count = 0
        self.consecutive_csrt_failures = 0
        
        logging.info(f"[HYBRID] Tracking started: {detection['class_name']} "
                    f"({detection['confidence']:.2f})")
        return True
    
    def select_object_by_class(self, frame: np.ndarray, class_name: str) -> bool:
        """
        Auto-select objek berdasarkan class.
        Pilih yang confidence tertinggi.
        """
        detections = self._run_yolo_detection(frame)
        
        # Filter by class
        class_dets = [d for d in detections 
                     if d['class_name'] == class_name or d['class_id'] == class_name]
        
        if not class_dets:
            logging.warning(f"[HYBRID] No '{class_name}' detected")
            return False
        
        # Pilih confidence tertinggi
        best = max(class_dets, key=lambda x: x['confidence'])
        return self._start_tracking(frame, best)
    
    def select_nearest_to_center(self, frame: np.ndarray) -> bool:
        """Auto-select objek terdekat ke tengah frame."""
        detections = self._run_yolo_detection(frame)
        
        if not detections:
            return False
        
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        
        def distance_to_center(det):
            x, y, bw, bh = det['bbox']
            obj_center = (x + bw // 2, y + bh // 2)
            return ((obj_center[0] - center[0]) ** 2 + 
                    (obj_center[1] - center[1]) ** 2) ** 0.5
        
        nearest = min(detections, key=distance_to_center)
        return self._start_tracking(frame, nearest)
    
    def update(self, frame: np.ndarray) -> tuple:
        """
        Update tracker dengan frame baru.
        
        Logic:
        1. Jika frame_count % detect_interval == 0: Run YOLO
        2. Selainnya: Run CSRT
        3. Jika CSRT fail beberapa kali: Force YOLO re-detect
        
        Returns:
            tuple: (success, bbox, score)
        """
        if not self.is_tracking or self.csrt_tracker is None:
            return False, None, 0.0
        
        self.frame_count += 1
        
        # Determine apakah perlu YOLO detection
        need_yolo = (
            self.frame_count % self.detect_interval == 0 or
            self.consecutive_csrt_failures >= self.max_csrt_failures
        )
        
        if need_yolo:
            return self._update_with_yolo(frame)
        else:
            return self._update_with_csrt(frame)
    
    def _update_with_csrt(self, frame: np.ndarray) -> tuple:
        """Update menggunakan CSRT tracker."""
        success, bbox = self.csrt_tracker.update(frame)
        
        if success:
            # Validate bbox
            x, y, w, h = [int(v) for v in bbox]
            frame_h, frame_w = frame.shape[:2]
            
            if (w > 10 and h > 10 and 
                x >= 0 and y >= 0 and 
                x + w <= frame_w and y + h <= frame_h):
                
                self.last_valid_box = bbox
                self.consecutive_csrt_failures = 0
                self._csrt_quality = min(1.0, self._csrt_quality + 0.05)
                
                return True, bbox, self._csrt_quality
        
        # CSRT failed
        self.consecutive_csrt_failures += 1
        self._csrt_quality = max(0.0, self._csrt_quality - self._quality_decay * 2)
        
        logging.debug(f"[HYBRID] CSRT failed ({self.consecutive_csrt_failures}/{self.max_csrt_failures})")
        
        return False, None, self._csrt_quality
    
    def _update_with_yolo(self, frame: np.ndarray) -> tuple:
        """Update menggunakan YOLO detection + CSRT verification."""
        # Run YOLO
        detections = self._run_yolo_detection(frame)
        
        # Juga run CSRT untuk comparison
        csrt_success, csrt_bbox = self.csrt_tracker.update(frame)
        
        if csrt_success and csrt_bbox is not None:
            # Cari YOLO detection yang match dengan CSRT
            matching_det = self._find_matching_detection(csrt_bbox, detections)
            
            if matching_det:
                # YOLO confirms CSRT - update dengan YOLO bbox (lebih akurat)
                bbox = matching_det['bbox']
                
                # Re-init CSRT dengan bbox dari YOLO
                self.csrt_tracker = self._create_csrt()
                self.csrt_tracker.init(frame, bbox)
                
                self.last_valid_box = bbox
                self.consecutive_csrt_failures = 0
                self._csrt_quality = matching_det['confidence']
                
                logging.debug(f"[HYBRID] YOLO confirmed CSRT: {matching_det['class_name']}")
                return True, bbox, self._csrt_quality
            else:
                # CSRT ada result tapi YOLO tidak match
                # Trust CSRT untuk sekarang, tapi turunkan confidence
                self._csrt_quality = max(0.3, self._csrt_quality - 0.1)
                self.last_valid_box = csrt_bbox
                return True, csrt_bbox, self._csrt_quality
        
        # CSRT failed - coba re-detect dengan YOLO
        if self.last_valid_box is not None and detections:
            # Cari detection terdekat dengan posisi terakhir
            lx, ly, lw, lh = self.last_valid_box
            last_center = (lx + lw // 2, ly + lh // 2)
            
            def distance(det):
                x, y, w, h = det['bbox']
                center = (x + w // 2, y + h // 2)
                return ((center[0] - last_center[0]) ** 2 + 
                        (center[1] - last_center[1]) ** 2) ** 0.5
            
            # Filter by class jika ada
            candidates = detections
            if self.tracked_class is not None:
                candidates = [d for d in detections if d['class_id'] == self.tracked_class]
            
            if candidates:
                nearest = min(candidates, key=distance)
                
                # Check jika cukup dekat
                max_distance = max(lw, lh) * 2
                if distance(nearest) < max_distance:
                    # Re-init tracking dengan detection baru
                    bbox = nearest['bbox']
                    
                    self.csrt_tracker = self._create_csrt()
                    self.csrt_tracker.init(frame, bbox)
                    
                    self.last_valid_box = bbox
                    self.consecutive_csrt_failures = 0
                    self._csrt_quality = nearest['confidence']
                    
                    logging.info(f"[HYBRID] Re-detected: {nearest['class_name']}")
                    return True, bbox, self._csrt_quality
        
        # Completely lost
        self.consecutive_csrt_failures += 1
        return False, None, 0.0
    
    def release_target(self):
        """Release target dan reset state."""
        self.is_tracking = False
        self.csrt_tracker = None
        self.last_valid_box = None
        self.tracked_class = None
        self.frame_count = 0
        self.consecutive_csrt_failures = 0
        self._csrt_quality = 1.0
        logging.info("[HYBRID] Target released")
    
    def draw_detections(self, frame: np.ndarray, 
                       highlight_tracked: bool = True) -> np.ndarray:
        """Draw semua YOLO detections pada frame."""
        display = frame.copy()
        
        for det in self.all_detections:
            x, y, w, h = det['bbox']
            
            # Highlight tracked object
            if (highlight_tracked and self.is_tracking and 
                self.tracked_class == det['class_id'] and
                self.last_valid_box is not None):
                
                iou = self._calculate_iou(self.last_valid_box, det['bbox'])
                if iou > 0.3:
                    color = (0, 255, 0)  # Green - tracked
                    thickness = 3
                else:
                    color = (255, 255, 0)  # Cyan - same class
                    thickness = 1
            else:
                color = (255, 255, 0)  # Cyan
                thickness = 1
            
            cv2.rectangle(display, (x, y), (x + w, y + h), color, thickness)
            
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(display, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return display
    
    def get_status(self) -> dict:
        """Get tracker status untuk debugging."""
        return {
            'is_tracking': self.is_tracking,
            'frame_count': self.frame_count,
            'csrt_quality': self._csrt_quality,
            'csrt_failures': self.consecutive_csrt_failures,
            'tracked_class': self.COCO_CLASSES.get(self.tracked_class, 'Unknown'),
            'detect_interval': self.detect_interval,
            'num_detections': len(self.all_detections)
        }
