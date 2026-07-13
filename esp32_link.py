"""
Pi <-> ESP32 serial connection

two packet types share the link:
- 0x01 flight state, Pi -> ESP32
- 0x02 PWM capture,  ESP32 -> Pi
"""

import struct

import serial

import hil_config as cfg
from uart_bridge import build_aircraft_state_packet, crc8


class Esp32Link:
    """
    opening serial port
    """

    def __init__(self):
        self._ser = serial.Serial(cfg.SERIAL_PORT, cfg.PI_ESP32_BAUD, timeout=0)
        # bytes we've read but haven't yet assembled into a complete packet
        self._rx_buf = bytearray()

    def send_flight_state(self, state):
        """
        push the current AircraftState to ESP32
        """
        self._ser.write(build_aircraft_state_packet(state))

    def poll_pwm_capture(self):
        """
        non blocking: drain the serial buffer and return the newest complete PWM capture packet as a tuple of 5 microsecond pulse widths
        in cfg.PWM_CHANNEL_NAMES order, or None if nothing complete has arrived

        called once per loop tick
        partial packets are kept in self._rx_buf so bytes split across ticks still get parsed correctly
        """
        if self._ser.in_waiting:
            self._rx_buf += self._ser.read(self._ser.in_waiting)

        newest = None
        while True:
            pkt = self._try_extract_pwm_packet()
            if pkt is None:
                break
            newest = pkt
        return newest

    def _try_extract_pwm_packet(self):
        """
        look for one complete, CRC valid PWM frame at the front of the buffer
        either consumes a full packet or drops one byte proven to be junk
        """
        buf = self._rx_buf

        start_idx = buf.find(bytes([cfg.PACKET_START]))
        if start_idx == -1:
            buf.clear()
            return None
        if start_idx > 0:
            del buf[:start_idx] # drop leading junk before the start byte

        if len(buf) < 2:
            return None # haven't seen the type byte yet

        pkt_type = buf[1]
        if pkt_type == cfg.MSG_AIRCRAFT_STATE:
            pkt_len = cfg.AIRCRAFT_STATE_PACKET_LEN
        elif pkt_type == cfg.PI_ESP32_PACKET_TYPE_PWM_CAPTURE:
            pkt_len = cfg.PWM_CAPTURE_PACKET_LEN
        else:
            del buf[0:1] # not a real start byte, resync
            return None

        if len(buf) < pkt_len:
            return None # packet hasn't fully arrived yet

        frame = bytes(buf[:pkt_len])
        del buf[:pkt_len]

        if frame[-1] != cfg.PACKET_END:
            print("[esp32_link] dropped frame: bad end byte")
            return None

        payload = frame[1:-2] # type byte + data, matches the ESP32's crc range
        if crc8(payload) != frame[-2]:
            print("[esp32_link] dropped frame: bad crc")
            return None

        if pkt_type != cfg.PI_ESP32_PACKET_TYPE_PWM_CAPTURE:
            # a flight state frame looping back, or something else
            return None

        # payload[0] is the type byte again, the rest are the 5 widths
        return struct.unpack(cfg.PWM_CAPTURE_FORMAT, payload)[1:]

    def close(self):
        self._ser.close()