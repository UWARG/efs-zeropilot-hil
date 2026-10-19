from dataclasses import dataclass
import struct


UDP_HOST = "127.0.0.1"

# JSBSim publishes the full 10-field AircraftState here.
JSBSIM_STATE_PORT = 18001

# Legacy port used by the old Simulink-to-UART bridge path.
LEGACY_UART_BRIDGE_PORT = 18002

# JSBSim receives normalized control inputs here.
JSBSIM_CONTROL_PORT = 18000

# HIL/flight server receives PWM control inputs here
PWM_TARGET_PORT = 18005
PWM_TARGET_IP = UDP_HOST
HOST_IP = UDP_HOST
HOST_PORT = JSBSIM_CONTROL_PORT


SERIAL_PORT = "/dev/ttyACM0"
PI_ESP32_BAUD = 115200

PACKET_START = 0xAA
PACKET_END = 0x55
MSG_AIRCRAFT_STATE = 0x01

# Network byte order keeps Python and ESP32 decoding deterministic.
AIRCRAFT_STATE_UDP_FORMAT = ">10d"
AIRCRAFT_STATE_PACKET_FORMAT = ">B10i"
AIRCRAFT_STATE_PACKET_LEN = 1 + struct.calcsize(AIRCRAFT_STATE_PACKET_FORMAT) + 1 + 1

PI_ESP32_PACKET_TYPE_PWM_CAPTURE = 0x02
PWM_CAPTURE_FORMAT = ">B5H"
PWM_CAPTURE_PACKET_LEN = 1 + struct.calcsize(PWM_CAPTURE_FORMAT) + 1 + 1
PWM_CHANNEL_NAMES = ["elevator", "aileron", "rudder", "flap", "throttle"]
SERVO_MIN_US = 1000
SERVO_MID_US = 1500
SERVO_MAX_US = 2000

DEFAULT_LAT_DEG = 43.4723  # waterloo, used as JSBSim's initial position and the ESP32's default
DEFAULT_LON_DEG = -80.5449

@dataclass
class AircraftState:
    altitude_ft: float
    airspeed_kts: float
    pitch_deg: float
    roll_deg: float
    heading_deg: float
    latitude_deg: float
    longitude_deg: float
    g_force: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float

    @classmethod
    def from_jsbsim_values(cls, values):
        return cls(*values)

    def to_udp_payload(self):
        return struct.pack(AIRCRAFT_STATE_UDP_FORMAT, *self.as_tuple())

    def as_tuple(self):
        return (
            self.altitude_ft,
            self.airspeed_kts,
            self.pitch_deg,
            self.roll_deg,
            self.heading_deg,
            self.latitude_deg,
            self.longitude_deg,
            self.g_force,
            self.roll_rate_rad_s,
            self.pitch_rate_rad_s,
        )

    def to_fixed_packet_values(self):
        return (
            int(round(self.altitude_ft * 100.0)),
            int(round(self.airspeed_kts * 100.0)),
            int(round(self.pitch_deg * 100.0)),
            int(round(self.roll_deg * 100.0)),
            int(round(self.heading_deg * 100.0)),
            int(round(self.latitude_deg * 10000000.0)),
            int(round(self.longitude_deg * 10000000.0)),
            int(round(self.g_force * 1000.0)),
            int(round(self.roll_rate_rad_s * 1000.0)),
            int(round(self.pitch_rate_rad_s * 1000.0)),
        )
