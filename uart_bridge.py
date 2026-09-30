import socket
import select
import struct

from hil_config import (
    AIRCRAFT_STATE_PACKET_FORMAT,
    AIRCRAFT_STATE_UDP_FORMAT,
    JSBSIM_STATE_PORT,
    LEGACY_UART_BRIDGE_PORT,
    MSG_AIRCRAFT_STATE,
    PACKET_END,
    PACKET_START,
    PI_ESP32_BAUD,
    SERIAL_PORT,
    UDP_HOST,
    AircraftState,
)


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
    return crc


def build_aircraft_state_packet(state):
    payload = struct.pack(
        AIRCRAFT_STATE_PACKET_FORMAT,
        MSG_AIRCRAFT_STATE,
        *state.to_fixed_packet_values(),
    )
    return bytes([PACKET_START]) + payload + bytes([crc8(payload), PACKET_END])


def decode_aircraft_state(raw):
    if len(raw) == struct.calcsize(AIRCRAFT_STATE_UDP_FORMAT):
        return AircraftState.from_jsbsim_values(struct.unpack(AIRCRAFT_STATE_UDP_FORMAT, raw))

    raise ValueError(f"unexpected UDP payload size: {len(raw)} bytes")

