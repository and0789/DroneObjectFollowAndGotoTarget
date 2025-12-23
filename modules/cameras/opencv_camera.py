"""
OpenCV Camera Implementation
============================
Camera menggunakan OpenCV VideoCapture.
Mendukung webcam, USB camera, video file, dan RTSP stream.
"""

import cv2
import logging

from .base_camera import BaseCamera


class OpenCVCamera(BaseCamera):
    """
    OpenCV-based camera.
    
    Mendukung:
    - Webcam (index: 0, 1, 2, ...)
    - Video file (path ke file)
    - RTSP stream (rtsp://...)
    - HTTP stream (http://...)
    """
    
    def __init__(self, source=0):
        """
        Initialize OpenCV Camera.
        
        Args:
            source: Camera source
                   - int: Camera index (0 = default webcam)
                   - str: Path ke video file atau stream URL
        """
        super().__init__()
        self.source = source
        self.cap = None
    
    def open(self) -> bool:
        """Open camera/video source."""
        try:
            self.cap = cv2.VideoCapture(self.source)
            
            if not self.cap.isOpened():
                logging.error(f"[OpenCVCamera] Failed to open source: {self.source}")
                return False
            
            # Get properties
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            if self.fps <= 0:
                self.fps = 30.0
            
            self.is_opened = True
            logging.info(f"[OpenCVCamera] Opened: {self.source}")
            logging.info(f"[OpenCVCamera] Resolution: {self.width}x{self.height} @ {self.fps:.1f} FPS")
            
            return True
            
        except Exception as e:
            logging.error(f"[OpenCVCamera] Error opening camera: {e}")
            return False
    
    def read(self) -> tuple:
        """Read frame dari camera."""
        if self.cap is None or not self.is_opened:
            return False, None
        
        ret, frame = self.cap.read()
        return ret, frame
    
    def release(self):
        """Release camera."""
        if self.cap is not None:
            self.cap.release()
            self.is_opened = False
            logging.info("[OpenCVCamera] Released")
    
    def set_resolution(self, width: int, height: int) -> bool:
        """Set camera resolution."""
        if self.cap is None:
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # Verify
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.width = actual_w
        self.height = actual_h
        
        success = (actual_w == width and actual_h == height)
        
        if not success:
            logging.warning(f"[OpenCVCamera] Resolution set to {actual_w}x{actual_h} "
                          f"(requested: {width}x{height})")
        
        return success
    
    def set_fps(self, fps: float) -> bool:
        """Set camera FPS."""
        if self.cap is None:
            return False
        
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        return True
    
    def set_autofocus(self, enabled: bool) -> bool:
        """Enable/disable autofocus."""
        if self.cap is None:
            return False
        
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if enabled else 0)
        return True
    
    def set_exposure(self, value: float) -> bool:
        """Set exposure value."""
        if self.cap is None:
            return False
        
        self.cap.set(cv2.CAP_PROP_EXPOSURE, value)
        return True
