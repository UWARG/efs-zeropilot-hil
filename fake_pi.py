import serial, struct, time

ser = serial.Serial("COM3", 115200)
# same format as hil_config.py: >B10i, lat/lon x1e7, alt/speed/etc x100, g/rates x1000
vals = [int(2000*100), int(80*100), 0, 0, int(90*100),
        int(43.5000*1e7), int(-80.5000*1e7), # DIFFERENT from default on purpose
        1000, 0, 0]
payload = struct.pack(">B10i", 0x01, *vals)
crc = 0
for b in payload:
    crc ^= b
pkt = bytes([0xAA]) + payload + bytes([crc, 0x55]) # 44 bytes

while True:
    ser.write(pkt)
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("$GP"):
        print(line)
    time.sleep(0.01)