"""
PiCamera2 Implementation
========================
Camera menggunakan libcamera via PiCamera2 library.
Mode: MATCH RECORD_VIDEO (Default Config + Flip).
Disamakan dengan script record_video.py yang terbukti normal.
"""

import logging
import numpy as np
import time
import cv2          
import config

from .base_camera import BaseCamera

class PiCamera2Camera(BaseCamera):
    """
    PiCamera2-based camera untuk Raspberry Pi.
    """
    
    def __init__(self, camera_num: int = 0):
        super().__init__()
        self.camera_num = camera_num
        self.picam2 = None
        self.config = None
        
        self._target_width = config.CAMERA_WIDTH
        self._target_height = config.CAMERA_HEIGHT
        self._target_fps = 30.0
    
    def open(self) -> bool:
        """Open camera dengan konfigurasi default (mengikuti record_video.py)."""
        try:
            from picamera2 import Picamera2
            import libcamera
            
            logging.info(f"[PiCamera2] Initializing camera {self.camera_num}...")
            
            self.picam2 = Picamera2(camera_num=self.camera_num)
            
            controls_dict = {
                "FrameRate": self._target_fps,
                "ExposureTime": 0 
            }
            
            try:
                controls_dict["AfMode"] = libcamera.controls.AfModeEnum.Auto
            except Exception:
                pass 
            
            # === PERBAIKAN: HAPUS FORMAT EXPLICIT ===
            # Kita biarkan Picamera2 memilih default (biasanya YUV/RGB compatible)
            # Ini menyamakan perilaku dengan record_video.py Anda.
            self.config = self.picam2.create_video_configuration(
                main={"size": (self._target_width, self._target_height)}, # Hapus "format": "RGB888"
                controls=controls_dict,
                buffer_count=4
            )
            
            self.picam2.configure(self.config)
            self.picam2.start()
            
            time.sleep(0.5) 
            
            stream_config = self.picam2.camera_configuration()["main"]
            self.width = stream_config["size"][0]
            self.height = stream_config["size"][1]
            self.fps = self._target_fps
            
            self.is_opened = True
            logging.info(f"[PiCamera2] Camera opened: {self.width}x{self.height} @ {self.fps} FPS")
            
            return True
            
        except ImportError:
            logging.error("[PiCamera2] picamera2 library not installed!")
            return False
            
        except Exception as e:
            logging.error(f"[PiCamera2] Error opening camera: {e}")
            return False
    
    def read(self) -> tuple:
        """Read frame."""
        if self.picam2 is None or not self.is_opened:
            return False, None
        
        try:
            # 1. Ambil data
            frame = self.picam2.capture_array("main")
            
            # 2. KONVERSI RGB KE BGR
            # Karena record_video.py berhasil dengan ini, kita pertahankan.
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # 3. ROTASI 180 DERAJAT (Gunakan FLIP)
            # Menggunakan cv2.flip(-1) lebih efisien dan sama dengan script record_video.py
            # -1 artinya flip vertikal DAN horizontal (sama dengan rotasi 180)
            frame = cv2.flip(frame, -1)
            
            return True, frame
            
        except Exception as e:
            logging.error(f"[PiCamera2] Error reading frame: {e}")
            return False, None
    
    def release(self):
        """Release camera."""
        if self.picam2 is not None:
            self.picam2.stop()
            self.is_opened = False
            logging.info("[PiCamera2] Released")

    # --- Helper methods ---
    def set_resolution(self, width: int, height: int) -> bool:
        if self.picam2 is None: return False
        self.picam2.stop()
        self._target_width = width
        self._target_height = height
        return self.open()

    def set_autofocus(self, enabled: bool) -> bool:
        if self.picam2 is None: return False
        import libcamera
        try:
            if enabled:
                self.picam2.set_controls({"AfMode": libcamera.controls.AfModeEnum.Auto})
                self.picam2.set_controls({"AfTrigger": libcamera.controls.AfTriggerEnum.Start})
            else:
                self.picam2.set_controls({"AfMode": libcamera.controls.AfModeEnum.Manual})
            return True
        except: return False

    def get_metadata(self) -> dict:
        if self.picam2 is None: return {}
        try: return self.picam2.capture_metadata()
        except: return {}