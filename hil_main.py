"""
replaces rpihil.slx's run button and prev QUARC launcher blocks (flightserver.m, UartBridgeLauncher.m, PigpioLauncher.m)
"""

import socket
import struct
import subprocess
import sys
import time

import hil_config as cfg
from esp32_link import Esp32Link


FLIGHT_SERVER_CMD = ["python3", "-u", "flight_server.py"]
LOOP_HZ = 100.0 # matches flight_server.py's fdm.set_dt(0.01)
LOOP_DT = 1.0 / LOOP_HZ

# order must match flight_server.py
# elevator, aileron, rudder, flap, throttle
CHANNEL_IS_BIPOLAR = [True, True, True, False, False]


def pwm_us_to_normalized(us, bipolar):
    """
    convert a servo pulse width in microseconds to the range JSBSim expects
    """
    us = max(cfg.SERVO_MIN_US, min(cfg.SERVO_MAX_US, us))
    span = cfg.SERVO_MAX_US - cfg.SERVO_MIN_US
    if bipolar:
        return 2.0 * (us - cfg.SERVO_MIN_US) / span - 1.0
    return (us - cfg.SERVO_MIN_US) / span


class HilRunner:
    def __init__(self):
        self._running = True
        self._jsb_proc = None
        self._esp32 = None
        self._state_sock = None
        self._control_sock = None

    def start(self):
        print("[hil] launching flight_server.py...")
        self._jsb_proc = subprocess.Popen(
            FLIGHT_SERVER_CMD,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True,
        )
        # TODO: replace with an actual readiness check
        time.sleep(2.0)

        print("[hil] opening ESP32 link...")
        self._esp32 = Esp32Link()

        # receive AircraftState from flight_server.py
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.bind((cfg.UDP_HOST, cfg.JSBSIM_STATE_PORT))
        self._state_sock.setblocking(False)

        # send control values back to flight_server.py
        self._control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        self.start()
        print(f"[hil] entering loop at {LOOP_HZ} Hz")
        next_tick = time.perf_counter()

        try:
            while self._running:
                self._tick()
                next_tick += LOOP_DT
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.perf_counter() # fell behind, resync
        except KeyboardInterrupt:
            print() # clean newline after ^C
        finally:
            self.shutdown()

    def _tick(self):
        # forward the newest aircraft state out to the ESP32
        state = self._recv_latest_state()
        if state is not None:
            self._esp32.send_flight_state(state)
        
        # debugging
        # for checking esp32->pi
        #time.sleep(0.02)
        #echo = self._esp32._ser.read(self._esp32._ser.in_waiting or 1)
        #if echo:
            #print(f"[echo] {echo.hex()}")
            
        #raw = self._esp32._ser.read(self._esp32._ser.in_waiting or 1)
        #if raw:
            #print(f"[debug] raw bytes: {raw.hex()}")

        # forward the newest PWM capture from the ESP32 into JSBSim's controls
        pwm = self._esp32.poll_pwm_capture()
        if pwm is not None:
            #print(f"[hil] pwm is not none")
            print(f"[hil] PWM: {pwm}")
            controls = [
                pwm_us_to_normalized(us, bipolar)
                for us, bipolar in zip(pwm, CHANNEL_IS_BIPOLAR)
            ]
            self._control_sock.sendto(
                struct.pack("ddddd", *controls),
                (cfg.UDP_HOST, cfg.JSBSIM_CONTROL_PORT),
            )
        else:
            print(f"[hil] pwm is none")

    def _recv_latest_state(self):
        """
        drain the state socket, keep only the newest packet this tick (matches QUARC)
        """
        newest = None
        while True:
            try:
                data, _ = self._state_sock.recvfrom(4096)
                newest = data
            except BlockingIOError:
                break

        if newest is None:
            return None

        expected = struct.calcsize(cfg.AIRCRAFT_STATE_UDP_FORMAT)
        if len(newest) != expected:
            print(f"[hil] bad state packet: {len(newest)} bytes, expected {expected}")
            return None

        values = struct.unpack(cfg.AIRCRAFT_STATE_UDP_FORMAT, newest)
        return cfg.AircraftState.from_jsbsim_values(values)

    def shutdown(self):
        print("[hil] shutting down...")
        if self._esp32:
            self._esp32.close()
        if self._state_sock:
            self._state_sock.close()
        if self._control_sock:
            self._control_sock.close()
        if self._jsb_proc:
            self._jsb_proc.terminate()
            try:
                self._jsb_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._jsb_proc.kill()
        print("[hil] done.")


if __name__ == "__main__":
    HilRunner().run()
    sys.exit(0)
