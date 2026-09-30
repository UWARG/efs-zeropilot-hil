import socket
import time

from hil_config import JSBSIM_STATE_PORT, UDP_HOST, AircraftState


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for i in range(10):
    state = AircraftState(
        altitude_ft=2000.0 + i,
        airspeed_kts=55.0,
        pitch_deg=2.0,
        roll_deg=5.0,
        heading_deg=45.0,
        latitude_deg=43.4723,
        longitude_deg=-80.5449,
        g_force=1.0,
        roll_rate_rad_s=0.0,
        pitch_rate_rad_s=0.0,
    )
    sock.sendto(state.to_udp_payload(), (UDP_HOST, JSBSIM_STATE_PORT))
    print(f"Sent AircraftState packet {i + 1}")
    time.sleep(0.1)

print("Done")
