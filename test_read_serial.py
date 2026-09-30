import serial

from hil_config import PI_ESP32_BAUD, SERIAL_PORT


s = serial.Serial(SERIAL_PORT, PI_ESP32_BAUD)

while True:
    line = s.readline()
    print(line.decode(errors="replace"), end="")
