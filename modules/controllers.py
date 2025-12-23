import logging
import math
# Mengimport semua konstanta dari config agar logika tidak berubah
from config import *

# =============================================================================
# VELOCITY SMOOTHER
# =============================================================================
class VelocitySmoother:
    """Low-pass filter untuk smoothing velocity commands."""
    def __init__(self, alpha=VELOCITY_SMOOTHING):
        self.alpha = alpha
        self.smooth_forward = 0.0
        self.smooth_lateral = 0.0
        self.smooth_vertical = 0.0
        self.smooth_yaw = 0.0
    
    def smooth(self, forward, lateral, vertical, yaw):
        self.smooth_forward = self.alpha * forward + (1 - self.alpha) * self.smooth_forward
        self.smooth_lateral = self.alpha * lateral + (1 - self.alpha) * self.smooth_lateral
        self.smooth_vertical = self.alpha * vertical + (1 - self.alpha) * self.smooth_vertical
        self.smooth_yaw = self.alpha * yaw + (1 - self.alpha) * self.smooth_yaw
        return (self.smooth_forward, self.smooth_lateral, 
                self.smooth_vertical, self.smooth_yaw)
    
    def reset(self):
        self.smooth_forward = 0.0
        self.smooth_lateral = 0.0
        self.smooth_vertical = 0.0
        self.smooth_yaw = 0.0


# =============================================================================
# DISTANCE CONTROLLER (Forward/Backward)
# =============================================================================
class DistanceController:
    """
    Controller untuk distance berdasarkan bbox ratio.
    """
    def __init__(self):
        self.target_ratio = TARGET_BBOX_RATIO
    
    def set_target(self, ratio):
        self.target_ratio = ratio
        logging.info(f"[DISTANCE] Target ratio set: {ratio:.2%}")
    
    def compute_follow(self, current_ratio):
        """MODE FOLLOW: Maintain distance."""
        if self.target_ratio <= 0:
            return 0.0, "NO_TARGET", 0.0
        
        percent_change = (self.target_ratio - current_ratio) / self.target_ratio * 100
        
        if abs(percent_change) < FOLLOW_DEADZONE_PERCENT:
            return 0.0, "OPTIMAL", percent_change
        
        forward_vel = (percent_change / 100.0) * FOLLOW_SPEED_GAIN
        
        if forward_vel > 0:
            forward_vel = min(forward_vel, MAX_FORWARD_VELOCITY)
            status = "TOO_FAR"
        else:
            forward_vel = max(forward_vel, -MAX_BACKWARD_VELOCITY)
            status = "TOO_CLOSE"
        
        return forward_vel, status, percent_change
    
    def compute_goto(self, current_ratio):
        """MODE GOTO: Approach target."""
        if current_ratio >= GOTO_STOP_RATIO:
            return 0.0, "ARRIVED", 100.0
        
        progress = (current_ratio / GOTO_STOP_RATIO) * 100
        speed_range = GOTO_MAX_SPEED - GOTO_MIN_SPEED
        speed_factor = 1.0 - (progress / 100.0)
        forward_vel = GOTO_MIN_SPEED + (speed_range * speed_factor)
        forward_vel = max(forward_vel, GOTO_MIN_SPEED)
        
        return forward_vel, "APPROACHING", progress


# =============================================================================
# YAW CONTROLLER (Left/Right Rotation)
# =============================================================================
class YawController:
    """
    Controller untuk yaw berdasarkan posisi X target.
    """
    def __init__(self, frame_center_x):
        self.center_x = frame_center_x
    
    def compute(self, object_x):
        error_px = object_x - self.center_x
        
        if abs(error_px) < YAW_DEADZONE_PX:
            return 0.0, error_px, "CENTERED"
        
        # error positif (kanan) → yaw negatif (putar kanan di MAVLink)
        yaw_rate = -error_px * YAW_GAIN
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, yaw_rate))
        
        status = "TURN_RIGHT" if error_px > 0 else "TURN_LEFT"
        return yaw_rate, error_px, status


# =============================================================================
# ALTITUDE CONTROLLER (Up/Down)
# =============================================================================
class AltitudeController:
    """
    Controller untuk altitude berdasarkan posisi Y target di frame.
    """
    def __init__(self, frame_center_y):
        self.center_y = frame_center_y
        self.current_altitude = None
    
    def set_current_altitude(self, altitude):
        self.current_altitude = altitude
    
    def compute(self, object_y, tracking_mode):
        # Check if enabled
        if not ENABLE_ALTITUDE_CONTROL:
            return 0.0, 0, "DISABLED"
        
        # Check if only active in GOTO mode
        if ALTITUDE_ONLY_IN_GOTO and tracking_mode != MODE_GOTO:
            return 0.0, 0, "FOLLOW_MODE"
        
        # Calculate error
        error_px = object_y - self.center_y
        
        # Dead zone
        if abs(error_px) < ALTITUDE_DEADZONE_PX:
            return 0.0, error_px, "LEVEL"
        
        # Calculate vertical velocity
        vertical_vel = error_px * ALTITUDE_GAIN
        
        # Determine status and clamp
        if vertical_vel > 0:
            # Turun
            vertical_vel = min(vertical_vel, MAX_DESCENT_RATE)
            status = "DESCEND"
        else:
            # Naik
            vertical_vel = max(vertical_vel, -MAX_ASCENT_RATE)
            status = "ASCEND"
        
        # Safety check
        if self.current_altitude is not None and MIN_ALTITUDE > 0:
            if self.current_altitude <= MIN_ALTITUDE and vertical_vel > 0:
                vertical_vel = 0.0
                status = "MIN_ALT"
        
        return vertical_vel, error_px, status