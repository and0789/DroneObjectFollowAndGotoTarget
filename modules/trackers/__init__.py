"""
Trackers Module
===============
Modular tracker implementations untuk drone object tracking.

Available Trackers:
- DaSiamRPNTracker: Fast single object tracking menggunakan Siamese network (perlu model ONNX)
- CSRTTracker: OpenCV built-in tracker, ringan dan akurat (tidak perlu model eksternal)
- YOLOTracker: Object detection + tracking menggunakan YOLO (perlu ultralytics)

Usage:
    from trackers import get_tracker
    
    # Option 1: DaSiamRPN (default, perlu model ONNX)
    tracker = get_tracker('dasiamrpn')
    
    # Option 2: CSRT (ringan, OpenCV built-in)
    tracker = get_tracker('csrt')
    
    # Option 3: YOLO (detection + tracking)
    tracker = get_tracker('yolo', model_path='yolov8n.pt')
    
    # Common interface
    tracker.initialize()
    tracker.select_object(frame)
    success, bbox, score = tracker.update(frame)
    tracker.release_target()
"""

from .base_tracker import BaseTracker
from .dasiamrpn_tracker import DaSiamRPNTracker
from .csrt_tracker import CSRTTracker
from .yolo_tracker import YOLOTracker
from .hybrid_tracker import HybridTracker

__all__ = ['BaseTracker', 'DaSiamRPNTracker', 'CSRTTracker', 'YOLOTracker', 'HybridTracker', 'get_tracker']


def get_tracker(tracker_type: str = "dasiamrpn", **kwargs):
    """
    Factory function untuk mendapatkan tracker.
    
    Args:
        tracker_type: "dasiamrpn", "csrt", atau "yolo"
        **kwargs: Arguments untuk tracker constructor
        
    Returns:
        BaseTracker instance
    
    Examples:
        # DaSiamRPN (perlu model ONNX)
        tracker = get_tracker("dasiamrpn")
        
        # CSRT (ringan, built-in OpenCV)
        tracker = get_tracker("csrt")
        
        # YOLO dengan model custom
        tracker = get_tracker("yolo", model_path="yolov11n.pt", target_classes=['person'])
    """
    trackers = {
        "dasiamrpn": DaSiamRPNTracker,
        "csrt": CSRTTracker,
        "yolo": YOLOTracker,
        "hybrid": HybridTracker,
    }
    
    tracker_type = tracker_type.lower()
    
    if tracker_type not in trackers:
        available = list(trackers.keys())
        raise ValueError(f"Unknown tracker type: '{tracker_type}'. "
                        f"Available: {available}")
    
    return trackers[tracker_type](**kwargs)


def list_available_trackers() -> dict:
    """
    List semua tracker yang tersedia beserta deskripsinya.
    
    Returns:
        dict: {tracker_name: description}
    """
    return {
        "dasiamrpn": "DaSiamRPN - Fast Siamese network tracker (perlu model ONNX)",
        "csrt": "CSRT - OpenCV built-in tracker, ringan dan akurat",
        "yolo": "YOLO - Detection + Tracking (perlu ultralytics package)",
    }
