# HIL Pi <-> ESP32 Protocol

## Runtime Split

- Raspberry Pi: JSBSim simulation core and UDP/Pi-side bridge logic.
- ESP32: hardware peripheral emulator for ZeroPilot.

## Pi to ESP32: AircraftState

Serial settings:

- Baud: 115200
- Start byte: `0xAA`
- End byte: `0x55`
- CRC: XOR of payload bytes
- Endian: big-endian

Packet layout:

| Field | Type | Scale |
|---|---:|---:|
| msg_type | uint8 | `0x01` |
| altitude_ft | int32 | `value * 100` |
| airspeed_kts | int32 | `value * 100` |
| pitch_deg | int32 | `value * 100` |
| roll_deg | int32 | `value * 100` |
| heading_deg | int32 | `value * 100` |
| latitude_deg | int32 | `value * 1e7` |
| longitude_deg | int32 | `value * 1e7` |
| g_force | int32 | `value * 1000` |
| roll_rate_rad_s | int32 | `value * 1000` |
| pitch_rate_rad_s | int32 | `value * 1000` |

Total packet length: 44 bytes.

## ESP32 GPS Spoofing

The ESP32 generates:

- `$GPRMC`
- `$GPGGA`

GPS serial settings:

- Baud: 9600
- Format: 8N1
- Default placeholder pins in `hil_esp32.ino`: `GPS_TX_PIN=17`, `GPS_RX_PIN=16`

Don't know the actual Physical GPIO pins for esp32 connection with ZP board. should modify the placeholder pins once know the actual pin. 

