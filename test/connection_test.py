import sys
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections
    from collections.abc import MutableMapping
    setattr(collections, "MutableMapping", MutableMapping)

import dronekit
import time

# Replace with your STIL's MAVLink UDP endpoint
# DRONE_CONNECTION_STRING = "udpin:127.0.0.1:14550"
DRONE_CONNECTION_STRING = "/dev/ttyACM0"

# connection_string = '/dev/ttyAMA10'

baud_rate = 57600  # Typically used for serial; adjust if necessary

print("Connecting to STIL on: {}".format(DRONE_CONNECTION_STRING))
vehicle = dronekit.connect(ip=DRONE_CONNECTION_STRING, baud=57600, wait_ready=True)

# Confirm connection by accessing vehicle attributes
print("Vehicle Mode: {}".format(vehicle.mode.name))
print("Global Location: {}".format(vehicle.location.global_frame))

# Example: Change mode to GUIDED
# vehicle.mode = VehicleMode("GUIDED")
# time.sleep(2)
# print("Mode changed to: {}".format(vehicle.mode.name))

# Always remember to close the connection
vehicle.close()
