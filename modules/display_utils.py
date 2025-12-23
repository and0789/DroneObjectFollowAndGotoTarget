import cv2
from config import *

def print_controls(tracker_type):
    print("\n" + "=" * 60)
    print(" DRONE OBJECT TRACKING SYSTEM")
    print(" dengan ALTITUDE CONTROL")
    print("=" * 60)
    print(f" Tracker: {tracker_type.upper()}")
    print(f" Invert: FWD={INVERT_FORWARD} YAW={INVERT_YAW} ALT={INVERT_ALTITUDE}")
    print(f" Altitude Control: {'ON' if ENABLE_ALTITUDE_CONTROL else 'OFF'}")
    print("\n CONTROLS:")
    print(" [S]     : Select Target")
    if tracker_type in ["yolo", "hybrid"]:
        print(" [A]     : Auto-select nearest")
        print(" [P]     : Auto-select person")
    print(" [R]     : Release Target")
    print(" [1]     : Mode FOLLOW")
    print(" [2]     : Mode GOTO")
    print(" [SPACE] : Pause/Resume")
    print(" [Q]     : Quit")
    print("=" * 60 + "\n")

def draw_crosshair(display, center_x, center_y):
    """Draw crosshair at frame center untuk referensi altitude."""
    h, w = display.shape[:2]
    color = (50, 50, 50)
    
    # Horizontal line
    cv2.line(display, (0, center_y), (w, center_y), color, 1)
    # Vertical line  
    cv2.line(display, (center_x, 0), (center_x, h), color, 1)
    # Center circle
    cv2.circle(display, (center_x, center_y), 5, color, 1)

def draw_debug_panel(display, mode_str, target_ratio, current_ratio, 
                     forward_vel, yaw_rate, yaw_error_px,
                     vertical_vel, alt_error_px,
                     fwd_status, yaw_status, alt_status,
                     smooth_fwd, smooth_yaw, smooth_vert):
    """Draw comprehensive debug panel with altitude info."""
    
    # Panel background
    panel_h = 170
    cv2.rectangle(display, (5, 5), (480, panel_h), (0, 0, 0), -1)
    cv2.rectangle(display, (5, 5), (480, panel_h), (100, 100, 100), 1)
    
    y_offset = 22
    line_height = 18
    
    # Line 1: Mode
    cv2.putText(display, f"MODE: {mode_str}", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    y_offset += line_height
    
    # Line 2: BBox ratio
    cv2.putText(display, f"Target: {target_ratio:.2%} | Current: {current_ratio:.2%}",
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    y_offset += line_height
    
    # Line 3: Forward status
    fwd_color = (0, 255, 0) if fwd_status in ["OPTIMAL", "ARRIVED"] else (0, 165, 255)
    cv2.putText(display, f"FWD: {fwd_status} | Raw:{forward_vel:+.2f} Smooth:{smooth_fwd:+.2f} m/s",
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, fwd_color, 1)
    y_offset += line_height
    
    # Line 4: Yaw status
    yaw_color = (0, 255, 0) if yaw_status == "CENTERED" else (0, 165, 255)
    cv2.putText(display, f"YAW: {yaw_status} | Err:{yaw_error_px:+.0f}px Rate:{smooth_yaw:+.1f} deg/s",
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, yaw_color, 1)
    y_offset += line_height
    
    # Line 5: Altitude status
    if ENABLE_ALTITUDE_CONTROL:
        alt_color = (0, 255, 0) if alt_status == "LEVEL" else (0, 165, 255)
        if alt_status == "MIN_ALT":
            alt_color = (0, 0, 255)
        cv2.putText(display, f"ALT: {alt_status} | Err:{alt_error_px:+.0f}px Vel:{smooth_vert:+.2f} m/s",
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, alt_color, 1)
    else:
        cv2.putText(display, "ALT: DISABLED",
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)
    y_offset += line_height
    
    # Line 6: Direction indicators
    indicators = []
    if abs(smooth_fwd) > 0.05:
        indicators.append(">>> FWD" if smooth_fwd > 0 else "<<< BACK")
    if abs(smooth_yaw) > 1.0:
        indicators.append("<<< LEFT" if smooth_yaw > 0 else ">>> RIGHT")
    if abs(smooth_vert) > 0.02:
        indicators.append("vvv DOWN" if smooth_vert > 0 else "^^^ UP")
    
    if not indicators:
        indicators.append("=== HOLDING ===")
    
    cv2.putText(display, " | ".join(indicators),
               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    y_offset += line_height
    
    # Line 7: Config status
    invert_str = f"Inv: F={INVERT_FORWARD} Y={INVERT_YAW} A={INVERT_ALTITUDE}"
    cv2.putText(display, invert_str, (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)