"""
HUD (Heads-Up Display) Module
=============================
Professional-style overlay untuk drone tracking system.
"""

import cv2
import config


def draw_pro_hud(frame, source_name, is_paused, mode=1):
    """
    Draw professional HUD overlay pada frame.
    
    Args:
        frame: Input frame (will be modified in-place)
        source_name: Video source description ("CAM" or "FILE")
        is_paused: Boolean indicating pause state
        mode: Tracking mode (MODE_FOLLOW or MODE_GOTO)
    """
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    
    # === CENTER CROSSHAIR ===
    length, gap = 25, 12
    crosshair_color = (200, 200, 200)
    
    # Horizontal lines
    cv2.line(frame, (cx - gap - length, cy), (cx - gap, cy), crosshair_color, 1)
    cv2.line(frame, (cx + gap, cy), (cx + gap + length, cy), crosshair_color, 1)
    
    # Vertical lines
    cv2.line(frame, (cx, cy - gap - length), (cx, cy - gap), crosshair_color, 1)
    cv2.line(frame, (cx, cy + gap), (cx, cy + gap + length), crosshair_color, 1)
    
    # Center dot
    cv2.circle(frame, (cx, cy), 2, (0, 0, 255), -1)
    
    # === TOP BAR (Mode & Info) ===
    # Background
    cv2.rectangle(frame, (0, 0), (w, 35), (0, 0, 0), -1)
    cv2.line(frame, (0, 35), (w, 35), (50, 50, 50), 1)
    
    # Mode indicator
    if mode == config.MODE_FOLLOW:
        mode_text = "MODE: FOLLOW"
        mode_color = (0, 255, 255)  # Yellow
        mode_desc = "Maintain Distance"
    else:
        mode_text = "MODE: GOTO"
        mode_color = (0, 165, 255)  # Orange
        mode_desc = "Approach Target"
    
    cv2.putText(frame, mode_text, (10, 22), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
    cv2.putText(frame, f"({mode_desc})", (150, 22), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    
    # Distance control info
    ratio_text = f"Target Ratio: {config.TARGET_BBOX_RATIO:.0%}"
    cv2.putText(frame, ratio_text, (w - 180, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    # === BOTTOM STATUS BAR ===
    bar_height = 45
    cv2.rectangle(frame, (0, h - bar_height), (w, h), (0, 0, 0), -1)
    cv2.line(frame, (0, h - bar_height), (w, h - bar_height), (50, 50, 50), 1)
    
    # System status
    if is_paused:
        status = "PAUSED"
        status_color = (0, 165, 255)  # Orange
    else:
        status = "LIVE"
        status_color = (0, 255, 0)  # Green
    
    cv2.putText(frame, f"SYS: {status}", (15, h - 18), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
    
    # Source indicator
    cv2.putText(frame, f"SRC: {source_name}", (150, h - 18), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    
    # Controls hint
    controls_text = "[S]elect [R]elease [1]Follow [2]Goto [Q]uit"
    cv2.putText(frame, controls_text, (280, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
    
    # Recording indicator (only when not paused)
    if not is_paused:
        # Blinking effect (simple)
        import time
        if int(time.time() * 2) % 2 == 0:
            cv2.circle(frame, (w - 100, h - 22), 6, (0, 0, 255), -1)
        cv2.putText(frame, "REC", (w - 85, h - 18), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


def draw_tracking_info(frame, bbox_ratio, error_x, forward_vel, yaw_rate, mode):
    """
    Draw detailed tracking information panel.
    Biasanya dipanggil dari object_tracker, tapi bisa digunakan terpisah.
    
    Args:
        frame: Input frame
        bbox_ratio: Current bbox area / frame area
        error_x: Horizontal error in pixels
        forward_vel: Forward velocity command
        yaw_rate: Yaw rate command
        mode: Tracking mode
    """
    h, w = frame.shape[:2]
    
    # Panel background
    panel_w, panel_h = 280, 90
    panel_x, panel_y = 10, 45
    
    cv2.rectangle(frame, (panel_x, panel_y), 
                  (panel_x + panel_w, panel_y + panel_h), 
                  (0, 0, 0), -1)
    cv2.rectangle(frame, (panel_x, panel_y), 
                  (panel_x + panel_w, panel_y + panel_h), 
                  (80, 80, 80), 1)
    
    # Title
    mode_str = "FOLLOW" if mode == config.MODE_FOLLOW else "GOTO"
    cv2.putText(frame, f"TRACKING [{mode_str}]", (panel_x + 5, panel_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    # Bbox ratio bar
    bar_x = panel_x + 5
    bar_y = panel_y + 30
    bar_w = panel_w - 10
    bar_h = 12
    
    # Background bar
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                  (50, 50, 50), -1)
    
    # Target marker
    target_pos = int(bar_x + (config.TARGET_BBOX_RATIO / 0.3) * bar_w)
    target_pos = min(max(target_pos, bar_x), bar_x + bar_w)
    cv2.line(frame, (target_pos, bar_y - 3), (target_pos, bar_y + bar_h + 3),
             (0, 255, 255), 2)
    
    # Current value bar
    current_pos = int(bar_x + (bbox_ratio / 0.3) * bar_w)
    current_pos = min(max(current_pos, bar_x), bar_x + bar_w)
    
    # Color based on distance
    if bbox_ratio < config.TARGET_BBOX_RATIO * 0.7:
        bar_color = (0, 165, 255)  # Orange - too far
    elif bbox_ratio > config.TARGET_BBOX_RATIO * 1.3:
        bar_color = (0, 0, 255)  # Red - too close
    else:
        bar_color = (0, 255, 0)  # Green - good
    
    cv2.rectangle(frame, (bar_x, bar_y), (current_pos, bar_y + bar_h),
                  bar_color, -1)
    
    cv2.putText(frame, f"BBox: {bbox_ratio:.1%}", (bar_x, bar_y + bar_h + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    # Error and velocity info
    cv2.putText(frame, f"Err_X: {error_x:+4.0f}px", (panel_x + 5, panel_y + 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(frame, f"Fwd: {forward_vel:+.2f}m/s", (panel_x + 100, panel_y + 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(frame, f"Yaw: {yaw_rate:+.2f}r/s", (panel_x + 195, panel_y + 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)


def draw_distance_indicator(frame, bbox_ratio, target_ratio):
    """
    Draw visual distance indicator di sisi kanan frame.
    
    Args:
        frame: Input frame
        bbox_ratio: Current bbox area ratio
        target_ratio: Target bbox area ratio
    """
    h, w = frame.shape[:2]
    
    # Indicator bar position
    bar_x = w - 30
    bar_y = 60
    bar_w = 15
    bar_h = h - 120
    
    # Background
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                  (40, 40, 40), -1)
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                  (80, 80, 80), 1)
    
    # Scale: 0% at bottom, 30% at top
    max_ratio = 0.30
    
    # Target line
    target_y = int(bar_y + bar_h - (target_ratio / max_ratio) * bar_h)
    target_y = max(bar_y, min(bar_y + bar_h, target_y))
    cv2.line(frame, (bar_x - 5, target_y), (bar_x + bar_w + 5, target_y),
             (0, 255, 255), 2)
    cv2.putText(frame, "TGT", (bar_x - 30, target_y + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
    
    # Current level
    current_y = int(bar_y + bar_h - (bbox_ratio / max_ratio) * bar_h)
    current_y = max(bar_y, min(bar_y + bar_h, current_y))
    
    # Fill from bottom to current
    if bbox_ratio < target_ratio * 0.8:
        fill_color = (0, 165, 255)  # Orange - too far
    elif bbox_ratio > target_ratio * 1.2:
        fill_color = (0, 0, 255)  # Red - too close
    else:
        fill_color = (0, 255, 0)  # Green - good
    
    cv2.rectangle(frame, (bar_x + 2, current_y), 
                  (bar_x + bar_w - 2, bar_y + bar_h - 2),
                  fill_color, -1)
    
    # Labels
    cv2.putText(frame, "FAR", (bar_x - 5, bar_y + bar_h + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
    cv2.putText(frame, "NEAR", (bar_x - 10, bar_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
