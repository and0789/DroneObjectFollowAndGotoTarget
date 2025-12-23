"""
YOLO Tracker Implementation
===========================
Tracker menggunakan YOLO (You Only Look Once) untuk detection + tracking.
Mendukung YOLOv8 dan YOLOv5.

Requires: ultralytics package
Install: pip install ultralytics
"""

import cv2
import numpy as np
import logging
import time

from .base_tracker import BaseTracker

# Import config
import config


class YOLOTracker(BaseTracker):
    """
    YOLO-based Object Tracker.
    
    Fitur:
    - Object detection menggunakan YOLO
    - Re-identification ketika target hilang
    - Support multiple YOLO versions (v5, v8, v11)
    - Class filtering (hanya track class tertentu)
    
    Modes:
    1. DETECTION_ONLY: Detect semua objek, user pilih untuk track
    2. TRACK_BY_CLASS: Track objek berdasarkan class (misal: "person")
    3. TRACK_BY_ID: Track objek dengan ID tertentu (menggunakan built-in tracker)
    """
    
    # Supported YOLO classes (COCO dataset)
    COCO_CLASSES = {
        0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
        5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
        10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
        14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
        # ... add more as needed
    }
    
    def __init__(self, model_path: str = None, target_classes: list = None):
        """
        Initialize YOLO Tracker.
        
        Args:
            model_path: Path ke YOLO model (.pt file). 
                       Default: yolov8n.pt (akan auto-download)
            target_classes: List of class names atau IDs untuk di-track.
                           None = track semua class.
                           Contoh: ['person', 'car'] atau [0, 2]
        """
        super().__init__()
        
        self.model_path = model_path or "yolov8n.pt"
        self.target_classes = target_classes
        self.model = None
        self.model_available = False
        
        # Tracking state
        self.tracked_id = None          # ID objek yang di-track
        self.tracked_class = None       # Class objek yang di-track
        self.tracked_features = None    # Features untuk re-id
        self.last_detection_time = 0
        
        # Detection parameters
        self.confidence_threshold = 0.5
        self.iou_threshold = 0.45
        
        # Re-identification parameters
        self.max_lost_frames = 30       # Max frames sebelum reset tracking
        self.lost_frame_count = 0
        
        # All detections dari frame terakhir
        self.all_detections = []
    
    def initialize(self) -> bool:
        """Load YOLO model."""
        try:
            from ultralytics import YOLO
            
            logging.info(f"[YOLO] Loading model: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.model_available = True
            
            logging.info(f"[YOLO] Model loaded successfully")
            logging.info(f"[YOLO] Target classes: {self.target_classes or 'ALL'}")
            return True
            
        except ImportError:
            logging.error("[YOLO] ultralytics package not installed!")
            logging.error("[YOLO] Install with: pip install ultralytics")
            self.model_available = False
            return False
            
        except Exception as e:
            logging.error(f"[YOLO] Failed to load model: {e}")
            self.model_available = False
            return False
    
    def detect(self, frame: np.ndarray) -> list:
        """
        Run YOLO detection pada frame.
        
        Args:
            frame: Input frame
            
        Returns:
            list: List of detections, each = {
                'bbox': (x, y, w, h),
                'class_id': int,
                'class_name': str,
                'confidence': float,
                'track_id': int or None
            }
        """
        if not self.model_available:
            return []
        
        try:
            # Run inference dengan tracking
            results = self.model.track(
                frame, 
                persist=True, 
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            
            detections = []
            
            for result in results:
                if result.boxes is None:
                    continue
                
                boxes = result.boxes
                
                for i in range(len(boxes)):
                    # Get bbox (xyxy format) dan convert ke xywh
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    x1, y1, x2, y2 = xyxy
                    w = x2 - x1
                    h = y2 - y1
                    bbox = (int(x1), int(y1), int(w), int(h))
                    
                    # Get class dan confidence
                    class_id = int(boxes.cls[i].cpu().numpy())
                    confidence = float(boxes.conf[i].cpu().numpy())
                    class_name = self.COCO_CLASSES.get(class_id, f"class_{class_id}")
                    
                    # Get track ID jika ada
                    track_id = None
                    if boxes.id is not None:
                        track_id = int(boxes.id[i].cpu().numpy())
                    
                    # Filter by target classes jika di-set
                    if self.target_classes is not None:
                        if class_id not in self.target_classes and class_name not in self.target_classes:
                            continue
                    
                    detections.append({
                        'bbox': bbox,
                        'class_id': class_id,
                        'class_name': class_name,
                        'confidence': confidence,
                        'track_id': track_id
                    })
            
            self.all_detections = detections
            return detections
            
        except Exception as e:
            logging.error(f"[YOLO] Detection error: {e}")
            return []
    
    def select_object(self, frame: np.ndarray) -> bool:
        """
        Interactive object selection.
        Menampilkan semua deteksi dan user memilih dengan klik.
        """
        if not self.model_available:
            return False
        
        # Run detection
        detections = self.detect(frame)
        
        if not detections:
            logging.warning("[YOLO] No objects detected")
            return False
        
        # Tampilkan deteksi dengan nomor
        display = frame.copy()
        for i, det in enumerate(detections):
            x, y, w, h = det['bbox']
            color = (0, 255, 0)
            cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
            label = f"{i}: {det['class_name']} ({det['confidence']:.2f})"
            cv2.putText(display, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        cv2.putText(display, "Press number key (0-9) to select object, ESC to cancel",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("SELECT TARGET", display)
        
        while True:
            key = cv2.waitKey(0) & 0xFF
            
            if key == 27:  # ESC
                cv2.destroyWindow("SELECT TARGET")
                return False
            
            if ord('0') <= key <= ord('9'):
                idx = key - ord('0')
                if idx < len(detections):
                    cv2.destroyWindow("SELECT TARGET")
                    return self._start_tracking(frame, detections[idx])
        
        return False
    
    def select_object_by_class(self, frame: np.ndarray, class_name: str) -> bool:
        """
        Otomatis select objek berdasarkan class.
        Memilih objek dengan confidence tertinggi.
        
        Args:
            frame: Input frame
            class_name: Nama class yang ingin di-track (misal: 'person')
        """
        if not self.model_available:
            return False
        
        detections = self.detect(frame)
        
        # Filter by class
        class_detections = [d for d in detections 
                          if d['class_name'] == class_name or d['class_id'] == class_name]
        
        if not class_detections:
            logging.warning(f"[YOLO] No '{class_name}' detected")
            return False
        
        # Pilih yang confidence tertinggi
        best = max(class_detections, key=lambda x: x['confidence'])
        return self._start_tracking(frame, best)
    
    def select_nearest_to_center(self, frame: np.ndarray) -> bool:
        """
        Otomatis select objek yang paling dekat ke tengah frame.
        """
        if not self.model_available:
            return False
        
        detections = self.detect(frame)
        
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
    
    def _start_tracking(self, frame: np.ndarray, detection: dict) -> bool:
        """Internal: Start tracking objek tertentu."""
        self.tracked_id = detection.get('track_id')
        self.tracked_class = detection['class_id']
        self.last_valid_box = detection['bbox']
        
        x, y, w, h = detection['bbox']
        self.obj_w = w
        self.obj_h = h
        
        frame_h, frame_w = frame.shape[:2]
        self.initial_bbox_ratio = (w * h) / (frame_w * frame_h)
        
        self.is_tracking = True
        self.lost_frame_count = 0
        
        logging.info(f"[YOLO] Tracking started: {detection['class_name']} "
                    f"(ID: {self.tracked_id}, conf: {detection['confidence']:.2f})")
        return True
    
    def update(self, frame: np.ndarray) -> tuple:
        """
        Update tracker dengan frame baru.
        
        Returns:
            tuple: (success, bbox, score)
        """
        if not self.is_tracking:
            return False, None, 0.0
        
        # Run detection
        detections = self.detect(frame)
        
        # Cari objek yang di-track
        tracked_det = None
        
        # Prioritas 1: Cari by track_id
        if self.tracked_id is not None:
            for det in detections:
                if det.get('track_id') == self.tracked_id:
                    tracked_det = det
                    break
        
        # Prioritas 2: Cari by class + proximity
        if tracked_det is None and self.last_valid_box is not None:
            same_class = [d for d in detections if d['class_id'] == self.tracked_class]
            
            if same_class:
                # Cari yang paling dekat dengan posisi terakhir
                lx, ly, lw, lh = self.last_valid_box
                last_center = (lx + lw // 2, ly + lh // 2)
                
                def distance(det):
                    x, y, w, h = det['bbox']
                    center = (x + w // 2, y + h // 2)
                    return ((center[0] - last_center[0]) ** 2 + 
                            (center[1] - last_center[1]) ** 2) ** 0.5
                
                nearest = min(same_class, key=distance)
                
                # Threshold: tidak terlalu jauh dari posisi terakhir
                max_distance = max(lw, lh) * 2
                if distance(nearest) < max_distance:
                    tracked_det = nearest
                    self.tracked_id = nearest.get('track_id')
        
        if tracked_det is not None:
            self.last_valid_box = tracked_det['bbox']
            self.lost_frame_count = 0
            
            x, y, w, h = tracked_det['bbox']
            self.obj_w = w
            self.obj_h = h
            
            return True, tracked_det['bbox'], tracked_det['confidence']
        else:
            self.lost_frame_count += 1
            
            if self.lost_frame_count > self.max_lost_frames:
                logging.warning("[YOLO] Target lost for too long, stopping tracking")
                self.is_tracking = False
            
            return False, None, 0.0
    
    def release_target(self):
        """Release target dan reset state."""
        self.is_tracking = False
        self.tracked_id = None
        self.tracked_class = None
        self.last_valid_box = None
        self.lost_frame_count = 0
        logging.info("[YOLO] Target released")
    
    def draw_detections(self, frame: np.ndarray, 
                       highlight_tracked: bool = True) -> np.ndarray:
        """
        Draw semua deteksi pada frame.
        
        Args:
            frame: Input frame
            highlight_tracked: Highlight objek yang di-track
            
        Returns:
            Frame dengan deteksi digambar
        """
        display = frame.copy()
        
        for det in self.all_detections:
            x, y, w, h = det['bbox']
            
            # Warna berbeda untuk tracked object
            if (highlight_tracked and self.is_tracking and 
                det.get('track_id') == self.tracked_id):
                color = (0, 255, 0)  # Green untuk tracked
                thickness = 3
            else:
                color = (255, 255, 0)  # Cyan untuk lainnya
                thickness = 1
            
            cv2.rectangle(display, (x, y), (x + w, y + h), color, thickness)
            
            # Label
            label = f"{det['class_name']}"
            if det.get('track_id') is not None:
                label += f" #{det['track_id']}"
            label += f" {det['confidence']:.2f}"
            
            cv2.putText(display, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return display
