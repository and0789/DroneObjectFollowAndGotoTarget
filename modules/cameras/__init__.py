# modules/cameras/__init__.py

# Cek ketersediaan Picamera2
try:
    import picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False

# Import OpenCV Camera (Pasti ada)
from .opencv_camera import OpenCVCamera

# Import PiCamera2 Camera (Hanya jika library tersedia)
if PICAMERA2_AVAILABLE:
    # Pastikan di dalam picamera2_camera.py import config-nya sudah diperbaiki (import config)
    from .picamera2_camera import PiCamera2Camera

def get_camera(camera_type, **kwargs):
    if camera_type == 'opencv':
        return OpenCVCamera(**kwargs)
    elif camera_type == 'picamera2':
        if not PICAMERA2_AVAILABLE:
            raise ValueError("PiCamera2 library not installed or not supported on this device")
        return PiCamera2Camera(**kwargs)
    else:
        raise ValueError(f"Unknown camera type: {camera_type}")