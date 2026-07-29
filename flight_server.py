import jsbsim
import socket
import struct
import time
import os
from hil_config import AircraftState, JSBSIM_STATE_PORT, UDP_HOST
from hil_config import JSBSIM_CONTROL_PORT, HOST_IP
from hil_config import DEFAULT_LAT_DEG, DEFAULT_LON_DEG


TARGET_IP = UDP_HOST;        TARGET_PORT = JSBSIM_STATE_PORT  # Send full AircraftState

ROOT_DIR = "/home/pi/jsbsim_data" 
AIRCRAFT = "c172x"


fdm = jsbsim.FGFDMExec(ROOT_DIR)
fdm.load_model(AIRCRAFT)
fdm.set_dt(0.01)

fdm['ic/h-sl-ft'] = 2000
fdm['ic/vc-kts'] = 5
fdm['propulsion/engine[0]/set-running'] = 1
# was doing conversion near africa (0) placing at waterloo now just like hardcoded esp32 values
fdm['ic/lat-geod-deg'] = DEFAULT_LAT_DEG
fdm['ic/long-gc-deg'] = DEFAULT_LON_DEG
fdm.run_ic()


sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.bind((HOST_IP, JSBSIM_CONTROL_PORT))
sock_in.setblocking(False)

sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"--- JSBSim SERVER RUNNING ---")
print(f"Listening on {JSBSIM_CONTROL_PORT}")
print(f"Sending State to {TARGET_PORT}")

controls = [0.0, 0.0, 0.0, 0.0, 0.0] 

try:
    while True:
        try:
            data, addr = sock_in.recvfrom(1024)
            controls = struct.unpack('ddddd', data) 
        except BlockingIOError:
            pass 

        fdm['fcs/elevator-cmd-norm'] = controls[0] # -1.0 to 1.0
        fdm['fcs/aileron-cmd-norm']  = controls[1] # -1.0 to 1.0
        fdm['fcs/rudder-cmd-norm']   = controls[2] # -1.0 to 1.0
        fdm['fcs/flap-cmd-norm']     = controls[3] #  0.0 to 1.0
        fdm['fcs/throttle-cmd-norm'] = controls[4] #  0.0 to 1.0
        

        fdm.run()
        
        state = AircraftState.from_jsbsim_values([
            fdm['position/h-sl-ft'],       # 1. Alt
            fdm['velocities/vc-kts'],      # 2. Speed
            fdm['attitude/theta-deg'],     # 3. Pitch
            fdm['attitude/phi-deg'],       # 4. Roll
            fdm['attitude/psi-deg'],       # 5. Heading
            fdm['position/lat-geod-deg'],  # 6. Lat
            fdm['position/long-gc-deg'],   # 7. Lon
            fdm['accelerations/n-pilot-z-norm'], # 8. G-Force
            fdm['velocities/p-rad_sec'],   # 9. Roll Rate
            fdm['velocities/q-rad_sec']    # 10. Pitch Rate
        ])
        
        packet = state.to_udp_payload()
        sock_out.sendto(packet, (TARGET_IP, TARGET_PORT))
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")
    sock_in.close()
    sock_out.close()
