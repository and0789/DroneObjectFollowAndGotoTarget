# config.py
# ==========================================================
# Configuration untuk Drone Object Tracking System
# Optimized untuk Fixed Camera (Tanpa Gimbal) dan Raspberry Pi
# ==========================================================
import logging
import os

# === KONEKSI DAN SISTEM ===
DRONE_CONNECTION_STRING = "udp:127.0.0.1:14550"
# DRONE_CONNECTION_STRING = "/dev/ttyACM0"
TAKEOFF_ALTITUDE = 5  # meter

# === MODE TRACKING ===
MODE_FOLLOW = 1  # Maintain distance - ikuti objek dengan jarak konstan
MODE_GOTO = 2    # Approach object - dekati objek sampai menabrak

# ==========================================================
# INVERT CONTROLS - UNTUK MEMPERBAIKI ARAH YANG SALAH
# ==========================================================
# Set True jika arah terbalik dari yang diharapkan

# YAW: Jika drone berputar ke KANAN saat objek di KIRI (atau sebaliknya)
INVERT_YAW = True

# FORWARD: Jika drone MUNDUR saat seharusnya MAJU (atau sebaliknya)
INVERT_FORWARD = False

# ALTITUDE: Jika drone NAIK saat seharusnya TURUN (atau sebaliknya)
INVERT_ALTITUDE = False

# ==========================================================
# KONFIGURASI KAMERA
# ==========================================================
CAMERA_PITCH_DEG = 0  # 0 = tegak lurus ke depan
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Target posisi X objek di frame (untuk yaw control)
TARGET_X = CAMERA_WIDTH // 2
TARGET_Y = CAMERA_HEIGHT // 2

# ==========================================================
# MODE FOLLOW - DISTANCE CONTROL
# ==========================================================
TARGET_BBOX_RATIO = 0.07  # 7% dari frame area
FOLLOW_SPEED_GAIN = 2.0
FOLLOW_DEADZONE_PERCENT = 10  # 10% dari target ratio

# ==========================================================
# MODE GOTO - APPROACH CONTROL  
# ==========================================================
GOTO_STOP_RATIO = 0.30
GOTO_MAX_SPEED = 2.0    # m/s
GOTO_MIN_SPEED = 0.3    # m/s

# ==========================================================
# YAW CONTROL - BERBASIS POSISI X
# ==========================================================
YAW_GAIN = 0.15
YAW_DEADZONE_PX = 40
MAX_YAW_RATE = 25.0  # deg/s

# ==========================================================
# ALTITUDE CONTROL - BERBASIS POSISI Y (BARU!)
# ==========================================================
# Untuk kamera TANPA GIMBAL (fixed, menghadap horizontal)
#
# Logika:
# - Target di BAWAH frame → Drone terlalu TINGGI → TURUN
# - Target di ATAS frame  → Drone terlalu RENDAH → NAIK
# - Target di TENGAH      → Ketinggian OK       → HOVER
#
# Ini memungkinkan MODE GOTO untuk mendekati objek di ground level

# Enable/Disable altitude control
ENABLE_ALTITUDE_CONTROL = True

# Gain untuk altitude (m/s per pixel error)
# Semakin besar = semakin responsif
# Recommended: 0.002 - 0.005
ALTITUDE_GAIN = 0.003

# Dead zone dalam pixel (vertikal)
# Jika target dalam range ini dari center Y, tidak naik/turun
ALTITUDE_DEADZONE_PX = 50

# Maximum rates (m/s)
MAX_DESCENT_RATE = 0.5   # Max turun
MAX_ASCENT_RATE = 0.3    # Max naik (lebih lambat untuk safety)

# Safety: Minimum altitude (meter)
# Drone tidak akan turun di bawah ini
# Set 0 untuk disable (hati-hati!)
MIN_ALTITUDE = 1.5

# Hanya aktif di MODE GOTO?
# True  = Altitude control hanya aktif di MODE GOTO
# False = Aktif di kedua mode (FOLLOW dan GOTO)
ALTITUDE_ONLY_IN_GOTO = False

# ==========================================================
# VELOCITY LIMITS
# ==========================================================
MAX_FORWARD_VELOCITY = 2.0   # m/s
MAX_BACKWARD_VELOCITY = 1.5  # m/s
MAX_LATERAL_VELOCITY = 1.0   # m/s
MAX_VERTICAL_VELOCITY = 0.5  # m/s

# ==========================================================
# SMOOTHING
# ==========================================================
VELOCITY_SMOOTHING = 0.2

# ==========================================================
# SAFETY PARAMETERS
# ==========================================================
MAX_LOST_FRAMES = 50
RECOVERY_COOLDOWN_FRAMES = 5

# ==========================================================
# LOGGING
# ==========================================================
LOG_LEVEL = logging.INFO

# ==========================================================
# PATHS AND MODELS
# ==========================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# --- DaSiamRPN Models ---
MODEL_FILE = "models/dasiamrpn_model.onnx"
KERNEL_R1 = "models/dasiamrpn_kernel_r1.onnx"
KERNEL_CLS1 = "models/dasiamrpn_kernel_cls1.onnx"

MODEL_PATH = os.path.join(BASE_DIR, MODEL_FILE)
KERNEL_R1_PATH = os.path.join(BASE_DIR, KERNEL_R1)
KERNEL_CLS1_PATH = os.path.join(BASE_DIR, KERNEL_CLS1)

# --- YOLO & Hailo Models ---
YOLO_MODEL_FILE = "models/yolo11n.pt"
HAILO_MODEL_FILE = "models/yolov8n.hef"

YOLO_MODEL_PATH = os.path.join(BASE_DIR, YOLO_MODEL_FILE)
HAILO_MODEL_PATH = os.path.join(BASE_DIR, HAILO_MODEL_FILE)

OUTPUT_PREFIX = "tracking_"

# ==========================================================
# PID PARAMETERS (legacy - untuk backward compatibility)
# ==========================================================
PID_YAW_KP = 0.3
PID_YAW_KI = 0.0
PID_YAW_KD = 0.1

DISTANCE_KP = 12.0
DISTANCE_KD = 3.0
DISTANCE_DEADZONE_RATIO = 0.01

# ==========================================================
# TUNING GUIDE
# ==========================================================
"""
PANDUAN TUNING:

=== FORWARD/BACKWARD (Jarak) ===
1. Drone tidak maju/mundur:
   - Naikkan FOLLOW_SPEED_GAIN (misal 3.0)
   - Turunkan FOLLOW_DEADZONE_PERCENT (misal 5)

2. Arah terbalik:
   - Set INVERT_FORWARD = True

3. Oscillate maju-mundur:
   - Turunkan FOLLOW_SPEED_GAIN
   - Naikkan FOLLOW_DEADZONE_PERCENT

=== YAW (Rotasi Kiri/Kanan) ===
4. Arah yaw terbalik:
   - Set INVERT_YAW = True

5. Yaw terlalu lambat:
   - Naikkan YAW_GAIN dan MAX_YAW_RATE

6. Yaw oscillate:
   - Turunkan YAW_GAIN
   - Naikkan YAW_DEADZONE_PX

=== ALTITUDE (Naik/Turun) - BARU! ===
7. Altitude tidak responsif:
   - Naikkan ALTITUDE_GAIN (misal 0.005)
   - Turunkan ALTITUDE_DEADZONE_PX (misal 30)

8. Arah altitude terbalik:
   - Set INVERT_ALTITUDE = True

9. Altitude oscillate:
   - Turunkan ALTITUDE_GAIN
   - Naikkan ALTITUDE_DEADZONE_PX

10. Drone turun terlalu rendah:
    - Naikkan MIN_ALTITUDE (misal 2.0)

11. Hanya ingin altitude control di GOTO:
    - Set ALTITUDE_ONLY_IN_GOTO = True

=== MODE GOTO ===
12. Berhenti terlalu jauh:
    - Naikkan GOTO_STOP_RATIO (misal 0.35)

13. Menabrak target:
    - Turunkan GOTO_STOP_RATIO (misal 0.25)
"""
