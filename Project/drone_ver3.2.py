import time
import math
import pybullet as p
import pybullet_data
from pymavlink import mavutil

# ==========================================================
# SETTINGS
# ==========================================================
# If you changed the SITL out port to 14552, update this too.
# Recommended: use 14552 if 14551 gave permission errors.
MAVLINK_PORT = 14551  # change to 14552 if needed
MAVLINK_LISTEN = "udp:192.168.0.184:14551"


# Trail settings
TRAIL_COLOR = [1, 0, 0]   # red
TRAIL_WIDTH = 2
TRAIL_LIFETIME = 15       # seconds

# Camera settings
CAM_DIST = 25
CAM_YAW = 45
CAM_PITCH = -30
CAM_TARGET = [0, 0, 0]

# ==========================================================
# HOME POSITION (set automatically from first GPS packet)
# ==========================================================
HOME_LAT = None
HOME_LON = None
HOME_ALT = None

# ==========================================================
# HELPER FUNCTIONS
# ==========================================================
def latlon_to_local_meters(lat, lon, alt, home_lat, home_lon, home_alt):
    """
    Convert GPS lat/lon/alt to local XY meters relative to home.
    Flat-earth approximation (works fine for short distances).
    """
    R = 6378137.0  # Earth radius in meters

    d_lat = math.radians(lat - home_lat)
    d_lon = math.radians(lon - home_lon)

    # East (x) and North (y)
    x = R * d_lon * math.cos(math.radians(home_lat))
    y = R * d_lat
    z = alt - home_alt  # Up

    return x, y, z


def euler_to_quat(roll, pitch, yaw):
    """Convert roll/pitch/yaw to PyBullet quaternion."""
    return p.getQuaternionFromEuler([roll, pitch, yaw])


# ==========================================================
# CONNECT MAVLINK
# ==========================================================
print(f"[INFO] Connecting to MAVLink: {MAVLINK_LISTEN}")
mav = mavutil.mavlink_connection(MAVLINK_LISTEN)

print("[INFO] Waiting for heartbeat...")
mav.wait_heartbeat()
print("[INFO] ✅ Heartbeat received from SITL!")

# ==========================================================
# START PYBULLET
# ==========================================================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetDebugVisualizerCamera(
    cameraDistance=CAM_DIST,
    cameraYaw=CAM_YAW,
    cameraPitch=CAM_PITCH,
    cameraTargetPosition=CAM_TARGET
)

p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Drone object (simple cube)
drone_id = p.loadURDF("cube_small.urdf", [0, 0, 1])

print("[INFO] ✅ PyBullet running. Listening for MAVLink messages...")

# ==========================================================
# MAIN LOOP
# ==========================================================
prev_pos = None
msg_counter = 0

# Store last attitude
last_quat = p.getQuaternionFromEuler([0, 0, 0])

while True:
    msg = mav.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=False)

    if msg is None:
        p.stepSimulation()
        time.sleep(1/60)
        continue

    msg_counter += 1
    if msg_counter % 100 == 0:
        print(f"[INFO] Received {msg_counter} messages. Last: {msg.get_type()}")

    # -----------------------
    # POSITION UPDATE
    # -----------------------
    if msg.get_type() == "GLOBAL_POSITION_INT":
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # mm -> meters

        if HOME_LAT is None:
            HOME_LAT, HOME_LON, HOME_ALT = lat, lon, alt
            print(f"[INFO] ✅ Home set: lat={HOME_LAT:.7f}, lon={HOME_LON:.7f}, alt={HOME_ALT:.2f}m")

        x, y, z = latlon_to_local_meters(lat, lon, alt, HOME_LAT, HOME_LON, HOME_ALT)

        pb_pos = [x, y, max(z, 0.1)]
        p.resetBasePositionAndOrientation(drone_id, pb_pos, last_quat)

        if prev_pos is not None:
            p.addUserDebugLine(prev_pos, pb_pos, TRAIL_COLOR, TRAIL_WIDTH, lifeTime=TRAIL_LIFETIME)
        prev_pos = pb_pos

    # -----------------------
    # ORIENTATION UPDATE
    # -----------------------
    if msg.get_type() == "ATTITUDE":
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        last_quat = euler_to_quat(roll, pitch, yaw)

        current_pos = p.getBasePositionAndOrientation(drone_id)[0]
        p.resetBasePositionAndOrientation(drone_id, current_pos, last_quat)

    p.stepSimulation()
    time.sleep(1/60)
