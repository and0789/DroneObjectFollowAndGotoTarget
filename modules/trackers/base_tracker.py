"""
Base Tracker Interface
======================
Abstract base class untuk semua tracker implementations.
Semua tracker harus mengimplementasikan interface ini.
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseTracker(ABC):
    """
    Abstract base class untuk object trackers.
    
    Semua tracker (DaSiamRPN, YOLO, dll) harus inherit dari class ini
    dan mengimplementasikan semua abstract methods.
    """
    
    def __init__(self):
        self.is_tracking = False
        self.last_valid_box = None
        self.initial_bbox_ratio = 0.0
        self.obj_w = 0
        self.obj_h = 0
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize tracker dan load model.
        
        Returns:
            bool: True jika berhasil, False jika gagal
        """
        pass
    
    @abstractmethod
    def select_object(self, frame: np.ndarray) -> bool:
        """
        Select object untuk di-track.
        
        Args:
            frame: Frame dari kamera
            
        Returns:
            bool: True jika objek berhasil dipilih
        """
        pass
    
    @abstractmethod
    def update(self, frame: np.ndarray) -> tuple:
        """
        Update tracker dengan frame baru.
        
        Args:
            frame: Frame dari kamera
            
        Returns:
            tuple: (success: bool, bbox: tuple, score: float)
                   bbox = (x, y, w, h)
        """
        pass
    
    @abstractmethod
    def release_target(self):
        """Release target dan reset tracker state."""
        pass
    
    def get_bbox_center(self, bbox: tuple) -> tuple:
        """
        Hitung center point dari bounding box.
        
        Args:
            bbox: (x, y, w, h)
            
        Returns:
            tuple: (center_x, center_y)
        """
        x, y, w, h = bbox
        return (x + w // 2, y + h // 2)
    
    def get_bbox_ratio(self, bbox: tuple, frame_shape: tuple) -> float:
        """
        Hitung rasio bbox area terhadap frame area.
        
        Args:
            bbox: (x, y, w, h)
            frame_shape: (height, width, channels)
            
        Returns:
            float: Rasio bbox area / frame area
        """
        _, _, w, h = bbox
        frame_h, frame_w = frame_shape[:2]
        return (w * h) / (frame_w * frame_h)
    
    @property
    def is_ready(self) -> bool:
        """Check if tracker is ready to track."""
        return self.is_tracking
