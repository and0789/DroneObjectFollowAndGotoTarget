"""
Argument Helper - Complete Field Testing Options
================================================
Berbagai opsi untuk testing di lapangan dengan berbagai kondisi.
"""

import argparse
import logging
import platform
from cameras import PICAMERA2_AVAILABLE

def parse_args():
    parser = argparse.ArgumentParser(
        description="Drone Object Tracking System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
================================================================================
                          COMMAND REFERENCE
================================================================================

[1] SIMULATION MODE (Tanpa Drone)
---------------------------------
# Test tracking saja, tanpa koneksi drone
python3 main.py --no-drone --tracker csrt
python3 main.py --no-drone --tracker yolo --target-class person


[2] TRACKING ONLY MODE (Connect tapi tidak kontrol)
---------------------------------------------------
# Connect ke drone, tapi hanya tracking (tidak kirim command)
# Bagus untuk test tracking dengan drone terbang manual
python3 main.py --tracking-only --tracker csrt


[3] MANUAL TAKEOVER (Drone sudah terbang)
-----------------------------------------
# Drone diterbangkan manual, program langsung tracking
python3 main.py --skip-takeoff --tracker csrt

# Dengan RC Override (untuk ALT_HOLD mode)
python3 main.py --skip-takeoff --use-rc --tracker csrt

# Skip semua (takeoff & land) - full manual
python3 main.py --skip-takeoff --skip-land --tracker csrt

# Tanpa ganti mode (tetap di mode apapun drone sekarang)
python3 main.py --skip-takeoff --skip-mode-change --tracker csrt


[4] AUTO MODE (Program kontrol penuh)
-------------------------------------
# Full auto: set GUIDED → arm → takeoff → tracking → land
python3 main.py --tracker csrt

# Auto tapi skip land
python3 main.py --skip-land --tracker csrt


[5] GPS BERMASALAH (Gunakan ALT_HOLD)
-------------------------------------
# Drone di ALT_HOLD, kontrol via RC Override
python3 main.py --skip-takeoff --skip-mode-change --use-rc --tracker csrt

# Force ALT_HOLD mode
python3 main.py --skip-takeoff --force-mode alt_hold --use-rc --tracker csrt


[6] TRACKING MODE
-----------------
# FOLLOW mode - jaga jarak dari target
python3 main.py --skip-takeoff --mode follow --tracker csrt

# GOTO mode - dekati target sampai dekat
python3 main.py --skip-takeoff --mode goto --tracker csrt


[7] TRACKER OPTIONS
-------------------
# CSRT (ringan, bagus untuk Raspberry Pi)
python3 main.py --skip-takeoff --tracker csrt

# DaSiamRPN (lebih akurat)
python3 main.py --skip-takeoff --tracker dasiamrpn

# YOLO (dengan deteksi objek)
python3 main.py --skip-takeoff --tracker yolo

# YOLO dengan target class tertentu
python3 main.py --skip-takeoff --tracker yolo --target-class person
python3 main.py --skip-takeoff --tracker yolo --target-class car
python3 main.py --skip-takeoff --tracker yolo --target-class dog

# Hybrid (YOLO + CSRT)
python3 main.py --skip-takeoff --tracker hybrid --target-class person


[8] CAMERA OPTIONS
------------------
# PiCamera2 (default)
python3 main.py --skip-takeoff --camera picamera2

# USB Webcam
python3 main.py --skip-takeoff --camera opencv --source 0

# Video file (untuk testing)
python3 main.py --no-drone --camera opencv --source video.mp4


[9] CONTROL TUNING
------------------
# Disable altitude control
python3 main.py --skip-takeoff --no-altitude-control --tracker csrt

# Disable yaw control (forward only)
python3 main.py --skip-takeoff --no-yaw-control --tracker csrt

# Disable forward control (yaw only)
python3 main.py --skip-takeoff --no-forward-control --tracker csrt


[10] KOMBINASI UNTUK KONDISI LAPANGAN
-------------------------------------
# GPS OK, drone sudah terbang di GUIDED/LOITER
python3 main.py --skip-takeoff --skip-land --tracker csrt --mode follow

# GPS Bermasalah, drone di ALT_HOLD
python3 main.py --skip-takeoff --skip-land --skip-mode-change --use-rc --tracker csrt

# Test tracking tanpa gerakkan drone
python3 main.py --skip-takeoff --tracking-only --tracker csrt

# Full auto test (GPS OK, pre-arm OK)
python3 main.py --tracker csrt --mode follow


================================================================================
                          KEYBOARD CONTROLS
================================================================================
  S     : Select target (draw ROI)
  A     : Auto-select nearest object (YOLO/Hybrid)
  P     : Auto-select person (YOLO/Hybrid)
  R     : Release target
  1     : Switch to FOLLOW mode
  2     : Switch to GOTO mode
  G     : Force GUIDED mode
  L     : Force LOITER mode
  H     : Force ALT_HOLD mode
  SPACE : Pause/Resume tracking
  Q     : Quit program
================================================================================
        """
    )
    
    # ==========================================================================
    # TRACKER OPTIONS
    # ==========================================================================
    tracker_group = parser.add_argument_group('Tracker Options')
    tracker_group.add_argument('--tracker', type=str, default='csrt',
                        choices=['dasiamrpn', 'csrt', 'yolo', 'hybrid'],
                        help='Tracker type (default: csrt)')
    tracker_group.add_argument('--yolo-model', type=str, default='yolo11n.pt',
                        help='YOLO model path')
    tracker_group.add_argument('--target-class', type=str, default=None,
                        help='Target class untuk YOLO/Hybrid (person, car, dog, dll)')
    tracker_group.add_argument('--detect-interval', type=int, default=10,
                        help='YOLO detection interval untuk Hybrid tracker')
    
    # ==========================================================================
    # CAMERA OPTIONS
    # ==========================================================================
    camera_group = parser.add_argument_group('Camera Options')
    camera_group.add_argument('--camera', type=str, default='picamera2',
                        choices=['opencv', 'picamera2'],
                        help='Camera type (default: picamera2)')
    camera_group.add_argument('--source', default=0,
                        help='Source untuk OpenCV (0, 1, atau path video)')
    
    # ==========================================================================
    # DRONE CONNECTION OPTIONS
    # ==========================================================================
    drone_group = parser.add_argument_group('Drone Connection')
    drone_group.add_argument('--no-drone', action='store_true',
                        help='Simulation mode - tanpa koneksi drone')
    drone_group.add_argument('--connection', type=str, default=None,
                        help='Override connection string (e.g., /dev/ttyACM0, udp:127.0.0.1:14550)')
    
    # ==========================================================================
    # FLIGHT CONTROL OPTIONS
    # ==========================================================================
    flight_group = parser.add_argument_group('Flight Control')
    flight_group.add_argument('--skip-takeoff', action='store_true',
                        help='Skip arm/takeoff - drone sudah terbang manual')
    flight_group.add_argument('--skip-land', action='store_true',
                        help='Skip auto-land saat exit')
    flight_group.add_argument('--skip-mode-change', action='store_true',
                        help='Jangan ubah flight mode - tetap di mode sekarang')
    flight_group.add_argument('--force-mode', type=str, default=None,
                        choices=['guided', 'loiter', 'alt_hold', 'stabilize'],
                        help='Force set ke mode tertentu saat start')
    flight_group.add_argument('--use-rc', action='store_true',
                        help='Gunakan RC Override (untuk ALT_HOLD mode)')
    
    # ==========================================================================
    # TRACKING MODE OPTIONS
    # ==========================================================================
    mode_group = parser.add_argument_group('Tracking Mode')
    mode_group.add_argument('--mode', type=str, default='follow',
                        choices=['follow', 'goto'],
                        help='Initial tracking mode (default: follow)')
    mode_group.add_argument('--tracking-only', action='store_true',
                        help='Hanya tracking, tidak kirim command ke drone')
    
    # ==========================================================================
    # CONTROL OPTIONS
    # ==========================================================================
    control_group = parser.add_argument_group('Control Options')
    control_group.add_argument('--no-altitude-control', action='store_true',
                        help='Disable altitude control (tidak naik/turun)')
    control_group.add_argument('--no-yaw-control', action='store_true',
                        help='Disable yaw control (tidak putar)')
    control_group.add_argument('--no-forward-control', action='store_true',
                        help='Disable forward control (tidak maju/mundur)')
    
    # ==========================================================================
    # OUTPUT OPTIONS
    # ==========================================================================
    output_group = parser.add_argument_group('Output Options')
    output_group.add_argument('--no-record', action='store_true',
                        help='Jangan rekam video output')
    output_group.add_argument('--output-dir', type=str, default='output',
                        help='Directory untuk simpan recording')
    
    # Parse arguments
    args = parser.parse_args()
    
    # ==========================================================================
    # AUTO-DETECT PLATFORM
    # ==========================================================================
    if 'arm' in platform.machine() or 'aarch64' in platform.machine():
        logging.info("Detected ARM platform (Raspberry Pi)")
        if args.camera == 'picamera2' and not PICAMERA2_AVAILABLE:
            logging.warning("PiCamera2 not available, using OpenCV")
            args.camera = 'opencv'
    else:
        if args.camera == 'picamera2':
            logging.info("Not on Pi, using OpenCV")
            args.camera = 'opencv'
    
    # ==========================================================================
    # PRINT ACTIVE OPTIONS
    # ==========================================================================
    print("\n" + "="*50)
    print(" ACTIVE OPTIONS")
    print("="*50)
    print(f" Tracker      : {args.tracker.upper()}")
    print(f" Camera       : {args.camera}")
    print(f" Mode         : {args.mode.upper()}")
    print(f" No Drone     : {args.no_drone}")
    print(f" Skip Takeoff : {args.skip_takeoff}")
    print(f" Skip Land    : {args.skip_land}")
    print(f" Skip Mode Chg: {args.skip_mode_change}")
    print(f" Use RC       : {args.use_rc}")
    print(f" Tracking Only: {args.tracking_only}")
    if args.target_class:
        print(f" Target Class : {args.target_class}")
    if args.force_mode:
        print(f" Force Mode   : {args.force_mode.upper()}")
    print("="*50 + "\n")
    
    return args
