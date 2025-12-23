#!/usr/bin/env python3
"""
=============================================================================
GUIDED MODE TEST PROGRAM
=============================================================================
Program untuk menguji berbagai command di GUIDED mode.
Membantu diagnosa masalah GPS dan test kontrol drone.

PENGGUNAAN:
    python3 test_guided.py

CATATAN KEAMANAN:
    - Test di area terbuka dan aman
    - Selalu siap dengan RC untuk override
    - Mulai dengan test sederhana dulu
    
=============================================================================
"""

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time
import sys
import math
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

# =============================================================================
# CONFIGURATION
# =============================================================================
CONNECTION_STRING = "udp:127.0.0.1:14550"  # SITL
# CONNECTION_STRING = "/dev/ttyACM0"  # Real drone via USB
# CONNECTION_STRING = "/dev/ttyAMA0"  # Real drone via UART

# Flight Modes
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

# =============================================================================
# HELPER CLASS
# =============================================================================
class GuidedTester:
    def __init__(self, connection_string):
        logging.info(f"Connecting to {connection_string}...")
        self.vehicle = utility.mavlink_connection(device=connection_string)
        
        try:
            self.vehicle.wait_heartbeat(timeout=10)
            logging.info("✓ Heartbeat received!")
        except:
            logging.error("✗ No heartbeat - check connection")
            sys.exit(1)
        
        self.target_system = self.vehicle.target_system
        self.target_component = self.vehicle.target_component
        logging.info(f"✓ Connected to system {self.target_system}")
        
        # Get initial status
        self.print_status()
    
    def get_heartbeat(self, timeout=2):
        """Get heartbeat message."""
        return self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
    
    def get_gps(self, timeout=2):
        """Get GPS status."""
        return self.vehicle.recv_match(type='GPS_RAW_INT', blocking=True, timeout=timeout)
    
    def get_position(self, timeout=2):
        """Get current position."""
        return self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    
    def get_attitude(self, timeout=2):
        """Get current attitude."""
        return self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
    
    def get_ekf_status(self, timeout=2):
        """Get EKF status."""
        return self.vehicle.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
    
    def get_sys_status(self, timeout=2):
        """Get system status."""
        return self.vehicle.recv_match(type='SYS_STATUS', blocking=True, timeout=timeout)
    
    def print_status(self):
        """Print comprehensive status."""
        print("\n" + "="*60)
        print(" DRONE STATUS")
        print("="*60)
        
        # Heartbeat / Mode
        hb = self.get_heartbeat()
        if hb:
            mode = FLIGHT_MODES.get(hb.custom_mode, f"UNKNOWN({hb.custom_mode})")
            armed = "ARMED" if hb.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
            print(f" Mode     : {mode}")
            print(f" Armed    : {armed}")
        
        # GPS
        gps = self.get_gps()
        if gps:
            fix_types = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 
                        4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
            fix = fix_types.get(gps.fix_type, f"Unknown({gps.fix_type})")
            print(f" GPS Fix  : {fix}")
            print(f" Satellites: {gps.satellites_visible}")
            print(f" HDOP     : {gps.eph/100:.2f}")
            print(f" VDOP     : {gps.epv/100:.2f}")
        else:
            print(" GPS      : No data")
        
        # Position
        pos = self.get_position()
        if pos:
            print(f" Altitude : {pos.relative_alt/1000:.2f} m (relative)")
            print(f" Heading  : {pos.hdg/100:.1f}°")
        
        # EKF
        ekf = self.get_ekf_status()
        if ekf:
            flags = ekf.flags
            print(f" EKF Flags: {flags:#06x}")
            print(f"   - Attitude OK    : {'✓' if flags & 1 else '✗'}")
            print(f"   - Velocity Horiz : {'✓' if flags & 2 else '✗'}")
            print(f"   - Velocity Vert  : {'✓' if flags & 4 else '✗'}")
            print(f"   - Pos Horiz Rel  : {'✓' if flags & 8 else '✗'}")
            print(f"   - Pos Horiz Abs  : {'✓' if flags & 16 else '✗'}")
            print(f"   - Pos Vert Abs   : {'✓' if flags & 32 else '✗'}")
            print(f"   - Pos Vert AGL   : {'✓' if flags & 64 else '✗'}")
            print(f"   - Const Pos Mode : {'✓' if flags & 128 else '✗'}")
            print(f"   - Pred Horiz Rel : {'✓' if flags & 256 else '✗'}")
            print(f"   - Pred Horiz Abs : {'✓' if flags & 512 else '✗'}")
        
        print("="*60 + "\n")
    
    def check_guided_requirements(self):
        """Check if GUIDED mode requirements are met."""
        print("\n" + "="*60)
        print(" GUIDED MODE REQUIREMENTS CHECK")
        print("="*60)
        
        issues = []
        
        # Check GPS
        gps = self.get_gps()
        if gps:
            if gps.fix_type < 3:
                issues.append(f"✗ GPS Fix insufficient (need 3D, got {gps.fix_type})")
            else:
                print(f"✓ GPS Fix OK ({gps.fix_type})")
            
            if gps.satellites_visible < 6:
                issues.append(f"✗ Satellites low ({gps.satellites_visible}, need 6+)")
            else:
                print(f"✓ Satellites OK ({gps.satellites_visible})")
            
            if gps.eph > 200:  # HDOP > 2.0
                issues.append(f"✗ HDOP too high ({gps.eph/100:.2f}, need < 2.0)")
            else:
                print(f"✓ HDOP OK ({gps.eph/100:.2f})")
        else:
            issues.append("✗ No GPS data")
        
        # Check EKF
        ekf = self.get_ekf_status()
        if ekf:
            required_flags = 0x1F  # Attitude + Velocity + Position
            if (ekf.flags & required_flags) != required_flags:
                issues.append(f"✗ EKF not ready (flags: {ekf.flags:#06x})")
            else:
                print(f"✓ EKF OK (flags: {ekf.flags:#06x})")
        else:
            issues.append("✗ No EKF data")
        
        print("="*60)
        
        if issues:
            print("\n⚠ ISSUES FOUND:")
            for issue in issues:
                print(f"  {issue}")
            print("\nGUIDED mode may not work properly!")
            return False
        else:
            print("\n✓ All requirements met for GUIDED mode")
            return True
    
    # =========================================================================
    # MODE COMMANDS
    # =========================================================================
    
    def set_mode(self, mode_id, timeout=5):
        """Set flight mode."""
        mode_name = FLIGHT_MODES.get(mode_id, f"MODE_{mode_id}")
        logging.info(f"Setting mode to {mode_name}...")
        
        self.vehicle.mav.set_mode_send(
            self.target_system,
            utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        start = time.time()
        while (time.time() - start) < timeout:
            hb = self.get_heartbeat(1)
            if hb and hb.custom_mode == mode_id:
                logging.info(f"✓ Mode changed to {mode_name}")
                return True
            time.sleep(0.2)
        
        logging.warning(f"✗ Mode change timeout")
        return False
    
    def set_guided(self):
        return self.set_mode(4)
    
    def set_alt_hold(self):
        return self.set_mode(2)
    
    def set_loiter(self):
        return self.set_mode(5)
    
    def set_stabilize(self):
        return self.set_mode(0)
    
    def set_land(self):
        return self.set_mode(9)
    
    def set_rtl(self):
        return self.set_mode(6)
    
    # =========================================================================
    # ARM / DISARM
    # =========================================================================
    
    def arm(self, timeout=10):
        """Arm the drone."""
        logging.info("Arming...")
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        start = time.time()
        while (time.time() - start) < timeout:
            hb = self.get_heartbeat(1)
            if hb and (hb.base_mode & utility.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                logging.info("✓ Armed")
                return True
            time.sleep(0.2)
        
        logging.warning("✗ Arm timeout - check pre-arm conditions")
        return False
    
    def disarm(self):
        """Disarm the drone."""
        logging.info("Disarming...")
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        logging.info("Disarm command sent")
    
    def force_arm(self):
        """Force arm (bypass pre-arm checks) - DANGEROUS!"""
        logging.warning("Force arming - BYPASSING SAFETY CHECKS!")
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 
            1,      # arm
            21196,  # force arm magic number
            0, 0, 0, 0, 0
        )
        time.sleep(1)
    
    # =========================================================================
    # GUIDED MODE COMMANDS
    # =========================================================================
    
    def takeoff(self, altitude=5, timeout=30):
        """Takeoff to altitude."""
        logging.info(f"Takeoff to {altitude}m...")
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            dialect.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            altitude
        )
        
        target = altitude * 0.75
        start = time.time()
        
        while (time.time() - start) < timeout:
            pos = self.get_position()
            if pos:
                alt = pos.relative_alt / 1000.0
                logging.info(f"  Altitude: {alt:.1f}m")
                if alt >= target:
                    logging.info(f"✓ Reached {alt:.1f}m")
                    return True
            time.sleep(0.5)
        
        logging.warning("✗ Takeoff timeout")
        return False
    
    def land(self):
        """Land the drone."""
        logging.info("Landing...")
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            utility.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
    
    def send_velocity(self, vx, vy, vz, yaw_rate=0, duration=1):
        """
        Send velocity command (BODY frame).
        
        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Down velocity (m/s) - positive = down
            yaw_rate: Yaw rate (rad/s)
            duration: How long to send (seconds)
        """
        logging.info(f"Velocity: vx={vx:.1f} vy={vy:.1f} vz={vz:.1f} yaw={math.degrees(yaw_rate):.1f}°/s for {duration}s")
        
        type_mask = 0x05C7  # Use velocity + yaw_rate
        
        start = time.time()
        while (time.time() - start) < duration:
            msg = self.vehicle.mav.set_position_target_local_ned_encode(
                0,
                self.target_system,
                self.target_component,
                utility.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                0, 0, 0,           # position
                vx, vy, vz,        # velocity
                0, 0, 0,           # acceleration
                0,                 # yaw
                yaw_rate           # yaw_rate
            )
            self.vehicle.mav.send(msg)
            time.sleep(0.05)  # 20Hz
        
        # Stop
        self.send_velocity_once(0, 0, 0, 0)
    
    def send_velocity_once(self, vx, vy, vz, yaw_rate=0):
        """Send single velocity command."""
        type_mask = 0x05C7
        msg = self.vehicle.mav.set_position_target_local_ned_encode(
            0,
            self.target_system,
            self.target_component,
            utility.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0,
            yaw_rate
        )
        self.vehicle.mav.send(msg)
    
    def hover(self):
        """Stop and hover."""
        logging.info("Hover (zero velocity)")
        self.send_velocity_once(0, 0, 0, 0)
    
    def goto_position_relative(self, north, east, down, timeout=30):
        """
        Go to position relative to current location (NED frame).
        
        Args:
            north: Meters north (positive) or south (negative)
            east: Meters east (positive) or west (negative)  
            down: Meters down (positive) or up (negative)
        """
        logging.info(f"Goto relative: N={north}m E={east}m D={down}m")
        
        # Get current position
        pos = self.get_position()
        if not pos:
            logging.error("Cannot get current position")
            return False
        
        # Type mask for position control
        type_mask = 0x0DF8  # Use position only
        
        start = time.time()
        while (time.time() - start) < timeout:
            msg = self.vehicle.mav.set_position_target_local_ned_encode(
                0,
                self.target_system,
                self.target_component,
                utility.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Relative to body
                type_mask,
                north, east, down,  # position offset
                0, 0, 0,            # velocity
                0, 0, 0,            # acceleration
                0, 0                # yaw, yaw_rate
            )
            self.vehicle.mav.send(msg)
            time.sleep(0.1)
        
        self.hover()
        return True
    
    def yaw_to_heading(self, heading_deg, timeout=10):
        """
        Rotate to absolute heading.
        
        Args:
            heading_deg: Target heading in degrees (0-360, 0=North)
        """
        logging.info(f"Yaw to heading: {heading_deg}°")
        
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            dialect.MAV_CMD_CONDITION_YAW,
            0,
            heading_deg,  # target heading
            25,           # yaw speed deg/s
            1,            # direction: 1=CW, -1=CCW
            0,            # 0=absolute, 1=relative
            0, 0, 0
        )
        
        time.sleep(timeout)
    
    def yaw_relative(self, angle_deg, timeout=10):
        """
        Rotate relative to current heading.
        
        Args:
            angle_deg: Degrees to rotate (positive=CW, negative=CCW)
        """
        logging.info(f"Yaw relative: {angle_deg}°")
        
        direction = 1 if angle_deg >= 0 else -1
        
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            dialect.MAV_CMD_CONDITION_YAW,
            0,
            abs(angle_deg),  # angle
            25,              # speed deg/s
            direction,       # direction
            1,               # relative
            0, 0, 0
        )
        
        time.sleep(timeout)
    
    # =========================================================================
    # RC OVERRIDE (untuk ALT_HOLD)
    # =========================================================================
    
    def rc_override(self, roll=1500, pitch=1500, throttle=1500, yaw=1500, duration=1):
        """
        Send RC override.
        Values: 1000-2000, neutral=1500
        """
        logging.info(f"RC Override: R={roll} P={pitch} T={throttle} Y={yaw} for {duration}s")
        
        channels = [roll, pitch, throttle, yaw] + [65535] * 14
        
        start = time.time()
        while (time.time() - start) < duration:
            msg = dialect.MAVLink_rc_channels_override_message(
                self.target_system,
                self.target_component,
                *channels
            )
            self.vehicle.mav.send(msg)
            time.sleep(0.05)
        
        # Release
        self.rc_release()
    
    def rc_release(self):
        """Release RC override."""
        channels = [65535] * 18
        msg = dialect.MAVLink_rc_channels_override_message(
            self.target_system,
            self.target_component,
            *channels
        )
        self.vehicle.mav.send(msg)
    
    # =========================================================================
    # CLOSE
    # =========================================================================
    
    def close(self):
        """Close connection."""
        self.hover()
        self.rc_release()
        self.vehicle.close()
        logging.info("Connection closed")


# =============================================================================
# INTERACTIVE MENU
# =============================================================================
def print_menu():
    print("\n" + "="*60)
    print(" GUIDED MODE TEST MENU")
    print("="*60)
    print(" STATUS:")
    print("   0  - Print full status")
    print("   1  - Check GUIDED requirements")
    print("")
    print(" MODE CHANGE:")
    print("   g  - Set GUIDED mode")
    print("   a  - Set ALT_HOLD mode")
    print("   l  - Set LOITER mode")
    print("   s  - Set STABILIZE mode")
    print("   r  - Set RTL mode")
    print("")
    print(" ARM/DISARM:")
    print("   arm    - Arm drone")
    print("   disarm - Disarm drone")
    print("   force  - Force arm (DANGEROUS!)")
    print("")
    print(" FLIGHT (harus di GUIDED + Armed):")
    print("   takeoff - Takeoff 5m")
    print("   land    - Land")
    print("   hover   - Stop/Hover")
    print("")
    print(" VELOCITY TEST:")
    print("   vf  - Forward 0.5m/s for 2s")
    print("   vb  - Backward 0.5m/s for 2s")
    print("   vl  - Left 0.5m/s for 2s")
    print("   vr  - Right 0.5m/s for 2s")
    print("   vu  - Up 0.3m/s for 2s")
    print("   vd  - Down 0.3m/s for 2s")
    print("   yl  - Yaw left 20°/s for 2s")
    print("   yr  - Yaw right 20°/s for 2s")
    print("")
    print(" RC OVERRIDE TEST (untuk ALT_HOLD):")
    print("   rf  - RC Forward")
    print("   rb  - RC Backward")
    print("   rl  - RC Left")
    print("   rr  - RC Right")
    print("")
    print(" OTHER:")
    print("   q  - Quit")
    print("="*60)


def main():
    print("\n" + "="*60)
    print(" GUIDED MODE TEST PROGRAM")
    print("="*60)
    print(f" Connection: {CONNECTION_STRING}")
    print("="*60)
    
    try:
        tester = GuidedTester(CONNECTION_STRING)
    except Exception as e:
        logging.error(f"Failed to connect: {e}")
        return
    
    print_menu()
    
    try:
        while True:
            cmd = input("\nCommand> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '0':
                tester.print_status()
            elif cmd == '1':
                tester.check_guided_requirements()
            
            # Mode changes
            elif cmd == 'g':
                tester.set_guided()
            elif cmd == 'a':
                tester.set_alt_hold()
            elif cmd == 'l':
                tester.set_loiter()
            elif cmd == 's':
                tester.set_stabilize()
            elif cmd == 'r':
                tester.set_rtl()
            
            # Arm/Disarm
            elif cmd == 'arm':
                tester.arm()
            elif cmd == 'disarm':
                tester.disarm()
            elif cmd == 'force':
                confirm = input("Force arm BYPASSES SAFETY! Type 'yes' to confirm: ")
                if confirm == 'yes':
                    tester.force_arm()
            
            # Flight
            elif cmd == 'takeoff':
                alt = input("Altitude (default 5m): ").strip()
                alt = float(alt) if alt else 5
                tester.takeoff(alt)
            elif cmd == 'land':
                tester.land()
            elif cmd == 'hover':
                tester.hover()
            
            # Velocity tests
            elif cmd == 'vf':
                tester.send_velocity(0.5, 0, 0, duration=2)
            elif cmd == 'vb':
                tester.send_velocity(-0.5, 0, 0, duration=2)
            elif cmd == 'vl':
                tester.send_velocity(0, -0.5, 0, duration=2)
            elif cmd == 'vr':
                tester.send_velocity(0, 0.5, 0, duration=2)
            elif cmd == 'vu':
                tester.send_velocity(0, 0, -0.3, duration=2)
            elif cmd == 'vd':
                tester.send_velocity(0, 0, 0.3, duration=2)
            elif cmd == 'yl':
                tester.send_velocity(0, 0, 0, math.radians(-30), duration=2)
            elif cmd == 'yr':
                tester.send_velocity(0, 0, 0, math.radians(30), duration=2)
            
            # RC Override tests
            elif cmd == 'rf':
                tester.rc_override(pitch=1400, duration=2)  # Forward
            elif cmd == 'rb':
                tester.rc_override(pitch=1600, duration=2)  # Backward
            elif cmd == 'rl':
                tester.rc_override(roll=1400, duration=2)   # Left
            elif cmd == 'rr':
                tester.rc_override(roll=1600, duration=2)   # Right
            
            elif cmd == 'menu':
                print_menu()
            elif cmd == '':
                pass
            else:
                print(f"Unknown command: {cmd}")
                print("Type 'menu' for help")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    
    finally:
        tester.close()
        print("Done")


if __name__ == "__main__":
    main()
