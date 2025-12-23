"""
Main Application - Complete Field Testing Version
=================================================
Drone Object Tracking System dengan berbagai opsi untuk testing lapangan.

Jalankan dengan --help untuk melihat semua opsi:
    python3 main.py --help
"""

import cv2
import os
import sys
import time
import logging
import math

# =============================================================================
# PATH SETUP
# =============================================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODULES_DIR = os.path.join(BASE_DIR, 'modules')
sys.path.insert(0, MODULES_DIR)

# =============================================================================
# IMPORTS
# =============================================================================
try:
    from config import *
    from cameras import get_camera
    from trackers import get_tracker
    from drone_control import DroneController
    from hud import draw_pro_hud
    from utils import build_unique_output_path
    
    from args_helper import parse_args
    from display_utils import draw_debug_panel, draw_crosshair, print_controls
    from controllers import DistanceController, YawController, AltitudeController, VelocitySmoother
    
except ImportError as e:
    print(f"[ERROR] Cannot import module: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Setup logging
logging.basicConfig(level=LOG_LEVEL, format='[%(levelname)s] %(message)s')


# =============================================================================
# MAIN FUNCTION
# =============================================================================
def main():
    args = parse_args()
    print_controls(args.tracker)
    
    # === INITIALIZE CAMERA ===
    kwargs = {}
    if args.camera == 'opencv':
        try:
            kwargs['source'] = int(args.source)
        except (ValueError, TypeError):
            kwargs['source'] = args.source
    
    camera = get_camera(args.camera, **kwargs)
    if not camera.open():
        logging.error("Failed to open camera")
        sys.exit(1)
    
    cam_width, cam_height = camera.get_resolution()
    logging.info(f"Camera: {cam_width}x{cam_height}")
    
    # === INITIALIZE TRACKER ===
    if args.tracker == 'yolo':
        target_classes = [args.target_class] if args.target_class else None
        tracker = get_tracker('yolo', model_path=args.yolo_model, target_classes=target_classes)
    elif args.tracker == 'hybrid':
        target_classes = [args.target_class] if args.target_class else None
        tracker = get_tracker('hybrid', yolo_model=args.yolo_model, 
                             detect_interval=args.detect_interval, target_classes=target_classes)
    elif args.tracker == 'csrt':
        tracker = get_tracker('csrt')
    else:
        tracker = get_tracker('dasiamrpn')
    
    if not tracker.initialize():
        logging.error("Failed to initialize tracker")
        camera.release()
        sys.exit(1)
    
    logging.info(f"Tracker: {args.tracker.upper()}")
    
    # === INITIALIZE DRONE ===
    drone = None
    skip_land = getattr(args, 'skip_land', False)
    use_rc_override = getattr(args, 'use_rc', False)
    tracking_only = getattr(args, 'tracking_only', False)
    
    # Override connection string if provided
    connection_string = args.connection if args.connection else DRONE_CONNECTION_STRING
    
    if not args.no_drone:
        try:
            drone = DroneController(connection_string)
            
            # === HANDLE FLIGHT MODE ===
            if not getattr(args, 'skip_mode_change', False):
                # Force mode jika diminta
                if args.force_mode:
                    if args.force_mode == 'guided':
                        drone.set_mode_guided()
                    elif args.force_mode == 'loiter':
                        drone.set_mode_loiter()
                    elif args.force_mode == 'alt_hold':
                        drone.set_mode_alt_hold()
                    elif args.force_mode == 'stabilize':
                        drone.set_mode_stabilize()
                elif not getattr(args, 'skip_takeoff', False):
                    # Auto mode - coba set GUIDED
                    drone.try_set_controllable_mode()
            else:
                logging.info(f"Skipping mode change - Current: {drone.get_mode_name()}")
            
            # === HANDLE TAKEOFF ===
            if getattr(args, 'skip_takeoff', False):
                logging.info("=== MANUAL TAKEOVER MODE ===")
                logging.info(f"Current mode: {drone.get_mode_name()}")
                logging.info(f"Altitude: {drone.get_altitude():.1f}m")
                skip_land = True
            else:
                # Normal flow: arm dan takeoff
                logging.info("=== AUTO TAKEOFF MODE ===")
                
                if not drone.is_armed:
                    if not drone.arm_drone():
                        logging.warning("Failed to arm!")
                
                if drone.is_armed:
                    current_alt = drone.get_altitude()
                    if current_alt < 2.0:
                        drone.takeoff(TAKEOFF_ALTITUDE)
                    else:
                        logging.info(f"Already at {current_alt:.1f}m")
                        skip_land = True
                        
        except Exception as e:
            logging.error(f"Drone error: {e}")
            import traceback
            traceback.print_exc()
            drone = None
    else:
        logging.info("=== SIMULATION MODE (No Drone) ===")
    
    # === CHECK TRACKING ONLY MODE ===
    if tracking_only:
        logging.info("=== TRACKING ONLY MODE ===")
        logging.info("Drone connected but NOT sending commands")
    
    # === INITIALIZE CONTROLLERS ===
    distance_ctrl = DistanceController()
    yaw_ctrl = YawController(cam_width // 2)
    altitude_ctrl = AltitudeController(cam_height // 2)
    smoother = VelocitySmoother()
    
    # === CHECK CONTROL OPTIONS ===
    enable_forward = not getattr(args, 'no_forward_control', False)
    enable_yaw = not getattr(args, 'no_yaw_control', False)
    enable_altitude = not getattr(args, 'no_altitude_control', False) and ENABLE_ALTITUDE_CONTROL
    
    if not enable_forward:
        logging.info("Forward control DISABLED")
    if not enable_yaw:
        logging.info("Yaw control DISABLED")
    if not enable_altitude:
        logging.info("Altitude control DISABLED")
    
    # === VIDEO WRITER ===
    out = None
    save_path = None
    
    if not getattr(args, 'no_record', False):
        output_dir = os.path.join(BASE_DIR, args.output_dir if hasattr(args, 'output_dir') else 'output')
        os.makedirs(output_dir, exist_ok=True)
        save_path = build_unique_output_path("CAM", None, output_dir, OUTPUT_PREFIX)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(save_path, fourcc, 30.0, (cam_width, cam_height))
        logging.info(f"Recording to: {save_path}")
    else:
        logging.info("Recording DISABLED")
    
    # === STATE ===
    paused = False
    tracking_mode = MODE_FOLLOW if args.mode == 'follow' else MODE_GOTO
    lost_frames = 0
    
    # FPS counter
    fps_start = time.time()
    fps_count = 0
    current_fps = 0.0
    
    logging.info("="*50)
    logging.info(" READY - Press 'S' to select target")
    logging.info("="*50)
    
    try:
        while True:
            loop_start = time.time()
            
            # === READ FRAME ===
            success, frame = camera.read()
            if not success:
                logging.warning("Frame read failed")
                time.sleep(0.1)
                continue
            
            # === PAUSED ===
            if paused:
                cv2.putText(frame, "PAUSED", (cam_width//2 - 50, cam_height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow("Object Tracker", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    paused = False
                elif key == ord('q'):
                    break
                continue
            
            display = frame.copy()
            draw_crosshair(display, cam_width // 2, cam_height // 2)
            
            # === Update altitude ===
            if drone and enable_altitude:
                try:
                    altitude_ctrl.set_current_altitude(drone.get_altitude())
                except:
                    pass
            
            # === TRACKING ===
            if tracker.is_tracking:
                success, bbox, score = tracker.update(frame)
                
                if success and bbox is not None:
                    lost_frames = 0
                    
                    x, y, w, h = map(int, bbox)
                    center_x, center_y = tracker.get_bbox_center(bbox)
                    bbox_ratio = tracker.get_bbox_ratio(bbox, frame.shape)
                    target_ratio = distance_ctrl.target_ratio
                    
                    # === YAW CONTROL ===
                    if enable_yaw:
                        yaw_rate, yaw_error_px, yaw_status = yaw_ctrl.compute(center_x)
                        if INVERT_YAW:
                            yaw_rate = -yaw_rate
                    else:
                        yaw_rate, yaw_error_px, yaw_status = 0, 0, "DISABLED"
                    
                    # === DISTANCE CONTROL ===
                    if enable_forward:
                        if tracking_mode == MODE_FOLLOW:
                            forward_vel, fwd_status, _ = distance_ctrl.compute_follow(bbox_ratio)
                            mode_str = "FOLLOW"
                        else:
                            forward_vel, fwd_status, _ = distance_ctrl.compute_goto(bbox_ratio)
                            mode_str = "GOTO"
                        
                        if INVERT_FORWARD:
                            forward_vel = -forward_vel
                    else:
                        forward_vel, fwd_status = 0, "DISABLED"
                        mode_str = "FOLLOW" if tracking_mode == MODE_FOLLOW else "GOTO"
                    
                    # === ALTITUDE CONTROL ===
                    if enable_altitude:
                        vertical_vel, alt_error_px, alt_status = altitude_ctrl.compute(center_y, tracking_mode)
                        if INVERT_ALTITUDE:
                            vertical_vel = -vertical_vel
                    else:
                        vertical_vel, alt_error_px, alt_status = 0, 0, "DISABLED"
                    
                    # === SMOOTHING ===
                    smooth_fwd, smooth_lat, smooth_vert, smooth_yaw = smoother.smooth(
                        forward_vel, 0.0, vertical_vel, yaw_rate
                    )
                    
                    # === SEND TO DRONE ===
                    if drone and not tracking_only:
                        if use_rc_override:
                            # RC Override untuk ALT_HOLD
                            drone.send_rc_velocity(smooth_fwd, smooth_lat, smooth_vert, smooth_yaw)
                        else:
                            # Velocity command untuk GUIDED/LOITER
                            yaw_rate_rad = math.radians(smooth_yaw)
                            drone.send_velocity_command(smooth_fwd, smooth_lat, smooth_vert, yaw_rate_rad)
                    
                    # === DRAW ===
                    color = (0, 255, 0) if score > 0.6 else (0, 255, 150)
                    cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
                    cv2.circle(display, (int(center_x), int(center_y)), 5, color, -1)
                    
                    # Altitude error line
                    if enable_altitude:
                        cv2.line(display, (int(center_x), cam_height//2), 
                                (int(center_x), int(center_y)), (255, 0, 255), 2)
                    
                    # Debug panel
                    draw_debug_panel(display, mode_str, target_ratio, bbox_ratio,
                                    forward_vel, yaw_rate, yaw_error_px,
                                    vertical_vel, alt_error_px,
                                    fwd_status, yaw_status, alt_status,
                                    smooth_fwd, smooth_yaw, smooth_vert)
                else:
                    # Lost target
                    lost_frames += 1
                    if drone and not tracking_only:
                        if use_rc_override:
                            drone.send_rc_velocity(0, 0, 0, 0)
                        else:
                            smooth_fwd, _, smooth_vert, smooth_yaw = smoother.smooth(0, 0, 0, 0)
                            drone.send_velocity_command(smooth_fwd, 0, smooth_vert, math.radians(smooth_yaw))
                    
                    cv2.putText(display, f"SEARCHING... [{lost_frames}/{MAX_LOST_FRAMES}]",
                               (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    
                    if lost_frames > MAX_LOST_FRAMES:
                        tracker.release_target()
                        smoother.reset()
                        if drone and use_rc_override:
                            drone.release_rc_override()
            else:
                # Standby - no target
                if drone and not tracking_only:
                    if use_rc_override:
                        drone.release_rc_override()
                    else:
                        drone.send_velocity_command(0, 0, 0, 0)
                
                cv2.putText(display, "STANDBY - Press 'S' to select",
                           (cam_width//2 - 180, cam_height//2 + 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            
            # === Show status ===
            if tracking_only:
                cv2.putText(display, "TRACKING ONLY", (10, cam_height - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            if drone:
                mode_text = f"Drone: {drone.get_mode_name()}"
                if use_rc_override:
                    mode_text += " [RC]"
                cv2.putText(display, mode_text, (cam_width - 180, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # === YOLO Detections & HUD ===
            if args.tracker in ["yolo", "hybrid"] and hasattr(tracker, 'draw_detections'):
                display = tracker.draw_detections(display)
            
            draw_pro_hud(display, "CAM", paused, tracking_mode)
            
            # === FPS ===
            fps_count += 1
            if fps_count >= 10:
                elapsed = time.time() - fps_start
                current_fps = fps_count / elapsed if elapsed > 0 else 0
                fps_start = time.time()
                fps_count = 0
            cv2.putText(display, f"FPS: {current_fps:.1f}", (cam_width - 100, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # === DISPLAY & RECORD ===
            cv2.imshow("Object Tracker", display)
            if out:
                out.write(display)
            
            # === KEY HANDLING ===
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'): 
                break
            elif key == ord('s'):
                if tracker.select_object(frame):
                    distance_ctrl.set_target(tracker.initial_bbox_ratio)
                    smoother.reset()
                    lost_frames = 0
            elif key == ord('a') and args.tracker in ["yolo", "hybrid"]:
                if tracker.select_nearest_to_center(frame):
                    distance_ctrl.set_target(tracker.initial_bbox_ratio)
                    smoother.reset()
                    lost_frames = 0
            elif key == ord('p') and args.tracker in ["yolo", "hybrid"]:
                if tracker.select_object_by_class(frame, "person"):
                    distance_ctrl.set_target(tracker.initial_bbox_ratio)
                    smoother.reset()
                    lost_frames = 0
            elif key == ord('r'):
                tracker.release_target()
                smoother.reset()
                if drone and use_rc_override:
                    drone.release_rc_override()
            elif key == ord('1'):
                tracking_mode = MODE_FOLLOW
                logging.info("Mode: FOLLOW")
            elif key == ord('2'):
                tracking_mode = MODE_GOTO
                logging.info("Mode: GOTO")
            elif key == ord(' '):
                paused = not paused
            elif key == ord('g'):
                if drone:
                    logging.info("Setting GUIDED mode...")
                    drone.set_mode_guided()
            elif key == ord('l'):
                if drone:
                    logging.info("Setting LOITER mode...")
                    drone.set_mode_loiter()
            elif key == ord('h'):
                if drone:
                    logging.info("Setting ALT_HOLD mode...")
                    drone.set_mode_alt_hold()
            elif key == ord('t'):
                # Toggle tracking only
                tracking_only = not tracking_only
                logging.info(f"Tracking Only: {tracking_only}")
            
            # === TIMING ===
            elapsed = time.time() - loop_start
            sleep_time = max(0, (1.0 / 30.0) - elapsed)
            if sleep_time > 0: 
                time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    except Exception as e:
        logging.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        logging.info("Shutting down...")
        
        # Stop drone
        if drone:
            try:
                drone.hover()
                drone.release_rc_override()
            except:
                pass
        
        camera.release()
        if out:
            out.release()
        cv2.destroyAllWindows()
        
        # Land if needed
        if drone and not skip_land and not tracking_only:
            logging.info("Auto-landing...")
            drone.land_drone()
        
        if drone:
            drone.close_connection()
        
        if save_path:
            logging.info(f"Recording saved: {save_path}")
        
        logging.info("Done!")


if __name__ == "__main__":
    main()
