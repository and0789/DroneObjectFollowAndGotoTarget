"""
Base Camera Interface
=====================
Abstract base class untuk semua camera implementations.
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseCamera(ABC):
    """
    Abstract base class untuk camera sources.
    
    Semua camera (OpenCV, PiCamera2, dll) harus inherit dari class ini.
    """
    
    def __init__(self):
        self.is_opened = False
        self.width = 0
        self.height = 0
        self.fps = 30.0
    
    @abstractmethod
    def open(self) -> bool:
        """
        Open camera connection.
        
        Returns:
            bool: True jika berhasil
        """
        pass
    
    @abstractmethod
    def read(self) -> tuple:
        """
        Read frame dari camera.
        
        Returns:
            tuple: (success: bool, frame: np.ndarray)
        """
        pass
    
    @abstractmethod
    def release(self):
        """Release camera resources."""
        pass
    
    @abstractmethod
    def set_resolution(self, width: int, height: int) -> bool:
        """
        Set camera resolution.
        
        Args:
            width: Frame width
            height: Frame height
            
        Returns:
            bool: True jika berhasil
        """
        pass
    
    def get_resolution(self) -> tuple:
        """
        Get current resolution.
        
        Returns:
            tuple: (width, height)
        """
        return (self.width, self.height)
    
    def get_fps(self) -> float:
        """Get camera FPS."""
        return self.fps
    
    @property
    def is_ready(self) -> bool:
        """Check if camera is ready."""
        return self.is_opened
