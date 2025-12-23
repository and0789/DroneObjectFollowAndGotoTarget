"""
Drone Control Module - Manual Takeover Compatible
==================================================
Untuk skenario:
- Drone diterbangkan MANUAL (ALT_HOLD/LOITER)
- Program hanya kirim velocity commands
- Pilot tetap bisa override dengan RC

Fitur:
- Tidak paksa mode change
- Timeout untuk semua operasi
- Support ALT_HOLD mode dengan RC override
"""

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time
import logging


# ArduCopter Flight Modes
FLIGHT_MODES = {
    0: "STABILIZE",
    1: "ACRO", 
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    16: "POSHOLD",
}


class DroneController:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        """
        Initialize drone controller.
        """
        self.current_mode = None
        self.current_altitude = 0.0
        self.is_armed = False
        
        # Connect to the vehicle
        logging.info(f"Connecting to {connection_string}...")
        
        try:
            self.vehicle = utility.mavlink_connection(device=connection_string)
            self.vehicle.wait_heartbeat(timeout=10)
            logging.info("Heartbeat received!")
        except Exception as e:
            logging.error(f"Failed to connect: {e}")
            raise ConnectionError(f"Cannot connect to drone: {e}")

        logging.info(
            f"Connected to system: {self.vehicle.target_system}, "
            f"component: {self.vehicle.target_component}"
        )
        
        # Get initial state
        self._update_state()

        # Channel defaults
        self.channels = {1: 1500, 2: 1500, 3: 1500, 4: 1500}
        self.min_pwm = 1000
        self.max_pwm = 2000

    def _update_state(self):
        """Update current mode and armed state."""
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            self.current_mode = msg.custom_mode
            self.is_armed = bool(msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode_name = FLIGHT_MODES.get(self.current_mode, f"UNKNOWN({self.current_mode})")
            logging.info(f"Current state: Mode={mode_name}, Armed={self.is_armed}")
            return True
        return False

    def get_mode_name(self, mode_id=None):
        """Get flight mode name."""
        if mode_id is None:
            mode_id = self.current_mode
        return FLIGHT_MODES.get(mode_id, f"UNKNOWN({mode_id})")

    def get_altitude(self):
        """Get current altitude in meters."""
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            self.current_altitude = msg.relative_alt / 1000.0
        return self.current_altitude

    def is_flying(self):
        """Check if drone is armed and in the air."""
        self._update_state()
        alt = self.get_altitude()
        return self.is_armed and alt > 0.5

    # =========================================================================
    # MODE CHANGE FUNCTIONS (dengan timeout, tidak blocking)
    # =========================================================================
    
    def set_mode(self, mode_id, mode_name="", timeout=5):
        """Set flight mode dengan timeout."""
        if not mode_name:
            mode_name = FLIGHT_MODES.get(mode_id, f"MODE_{mode_id}")
        
        logging.info(f"Requesting mode: {mode_name}...")
        
        base_mode = utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            base_mode,
            mode_id
        )
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                self.current_mode = msg.custom_mode
                if msg.custom_mode == mode_id:
                    logging.info(f"Mode changed to {mode_name}!")
                    return True
            time.sleep(0.2)
        
        current = self.get_mode_name()
        logging.warning(f"Mode change timeout. Current: {current}")
        return False

    def set_mode_guided(self):
        return self.set_mode(4, "GUIDED")

    def set_mode_alt_hold(self):
        return self.set_mode(2, "ALT_HOLD")

    def set_mode_loiter(self):
        return self.set_mode(5, "LOITER")

    def set_mode_stabilize(self):
        return self.set_mode(0, "STABILIZE")

    def set_mode_land(self):
        return self.set_mode(9, "LAND")

    def try_set_controllable_mode(self):
        """
        Coba set ke mode yang bisa dikontrol.
        Return True jika berhasil atau sudah di mode yang ok.
        """
        self._update_state()
        
        # Mode yang bisa menerima velocity/position commands
        good_modes = [4, 5, 16]  # GUIDED, LOITER, POSHOLD
        
        if self.current_mode in good_modes:
            logging.info(f"Already in good mode: {self.get_mode_name()}")
            return True
        
        # Coba GUIDED
        logging.info("Trying to set GUIDED mode...")
        if self.set_mode_guided():
            return True
        
        # Coba LOITER
        logging.info("GUIDED failed, trying LOITER...")
        if self.set_mode_loiter():
            return True
        
        # Tetap lanjut meski gagal
        logging.warning(f"Could not change mode. Current: {self.get_mode_name()}")
        logging.warning("Program will continue - use RC to change mode if needed")
        return False

    # =========================================================================
    # ARM / DISARM (dengan timeout)
    # =========================================================================
    
    def arm_drone(self, timeout=10):
        """Arm drone dengan timeout."""
        logging.info("Arming drone...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and (msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logging.info("Drone armed!")
                self.is_armed = True
                return True
            time.sleep(0.2)
        
        logging.warning("Arm timeout")
        return False

    def disarm_drone(self):
        """Disarm drone."""
        logging.info("Disarming...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        self._update_state()

    # =========================================================================
    # TAKEOFF / LAND
    # =========================================================================
    
    def takeoff(self, target_altitude=5, timeout=30):
        """Takeoff dengan timeout."""
        logging.info(f"Takeoff to {target_altitude}m...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            dialect.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            target_altitude
        )
        
        target_reached = target_altitude * 0.75
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            alt = self.get_altitude()
            logging.info(f"Altitude: {alt:.1f}m / {target_altitude}m")
            if alt >= target_reached:
                logging.info("Target altitude reached!")
                return True
            time.sleep(0.5)
        
        logging.warning("Takeoff timeout")
        return False

    def land_drone(self, timeout=60):
        """Land drone."""
        logging.info("Landing...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            utility.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg:
                if not (msg.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    logging.info("Landed and disarmed")
                    return True
            time.sleep(1)
        
        logging.warning("Land timeout")
        return False

    # =========================================================================
    # VELOCITY CONTROL - Ini yang utama untuk tracking!
    # =========================================================================
    
    def send_velocity_command(self, vel_x, vel_y, vel_z, yaw_rate=0.0):
        """
        Kirim velocity command (BODY frame).
        
        Args:
            vel_x: Forward (m/s, + = maju)
            vel_y: Lateral (m/s, + = kanan)  
            vel_z: Vertical (m/s, + = turun di NED)
            yaw_rate: Yaw rate (rad/s)
        
        Note: Ini bekerja di GUIDED dan LOITER mode.
        Di ALT_HOLD, gunakan RC override.
        """
        type_mask = 0x05C7  # Use velocity + yaw_rate
        
        msg = self.vehicle.mav.set_position_target_local_ned_encode(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            utility.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,           # position (ignored)
            vel_x, vel_y, vel_z,  # velocity
            0, 0, 0,           # acceleration (ignored)
            0,                 # yaw (ignored)
            yaw_rate           # yaw_rate
        )
        self.vehicle.mav.send(msg)

    def hover(self):
        """Stop - kirim zero velocity."""
        self.send_velocity_command(0, 0, 0, 0)

    # =========================================================================
    # RC OVERRIDE - Untuk mode ALT_HOLD
    # =========================================================================
    
    def send_rc_override(self, roll=1500, pitch=1500, throttle=1500, yaw=1500):
        """
        Kirim RC override.
        Nilai: 1000-2000, neutral = 1500
        
        Channel mapping:
        1 = Roll (kiri/kanan)
        2 = Pitch (maju/mundur)
        3 = Throttle (naik/turun)
        4 = Yaw (putar)
        """
        # Clamp values
        roll = max(1000, min(2000, roll))
        pitch = max(1000, min(2000, pitch))
        throttle = max(1000, min(2000, throttle))
        yaw = max(1000, min(2000, yaw))
        
        # 65535 = release/ignore channel
        channels = [roll, pitch, throttle, yaw] + [65535] * 14
        
        msg = dialect.MAVLink_rc_channels_override_message(
            self.vehicle.target_system,
            self.vehicle.target_component,
            *channels
        )
        self.vehicle.mav.send(msg)

    def send_rc_velocity(self, forward_vel, lateral_vel, vertical_vel, yaw_rate):
        """
        Convert velocity ke RC override PWM.
        Untuk digunakan di ALT_HOLD mode.
        
        Args:
            forward_vel: m/s (-2 to +2)
            lateral_vel: m/s (-2 to +2)
            vertical_vel: m/s (-1 to +1)
            yaw_rate: deg/s (-45 to +45)
        """
        # Convert velocity ke PWM offset dari neutral (1500)
        # Asumsi: ±500 PWM = ±full stick = ±2 m/s (untuk vel) atau ±45 deg/s (untuk yaw)
        
        # Pitch (channel 2) - forward/backward
        # Pitch DOWN (PWM < 1500) = maju
        pitch_offset = -forward_vel * 250  # 2 m/s = 500 PWM
        pitch = int(1500 + pitch_offset)
        
        # Roll (channel 1) - lateral
        # Roll RIGHT (PWM > 1500) = kanan
        roll_offset = lateral_vel * 250
        roll = int(1500 + roll_offset)
        
        # Throttle (channel 3) - vertical
        # Throttle UP (PWM > 1500) = naik
        # vertical_vel positif (NED down) = turun = throttle turun
        throttle_offset = -vertical_vel * 300  # ±1 m/s = ±300 PWM
        throttle = int(1500 + throttle_offset)
        
        # Yaw (channel 4)
        # Yaw RIGHT (PWM > 1500) = putar CW
        yaw_offset = yaw_rate * (500 / 45)  # 45 deg/s = 500 PWM
        yaw = int(1500 + yaw_offset)
        
        self.send_rc_override(roll, pitch, throttle, yaw)

    def release_rc_override(self):
        """Release semua RC override."""
        channels = [65535] * 18
        msg = dialect.MAVLink_rc_channels_override_message(
            self.vehicle.target_system,
            self.vehicle.target_component,
            *channels
        )
        self.vehicle.mav.send(msg)

    # =========================================================================
    # CONNECTION
    # =========================================================================
    
    def close_connection(self):
        """Close connection."""
        try:
            self.release_rc_override()
        except:
            pass
        try:
            self.vehicle.close()
        except:
            pass
        logging.info("Connection closed")
